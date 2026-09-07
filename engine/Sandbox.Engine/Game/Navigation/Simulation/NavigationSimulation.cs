using Sandbox.Navigation.Pathfinding;

namespace Sandbox.Navigation;

/// <summary>
/// Scene-independent simulation. All coordinates are in navigation space.
/// Commands and published state share the navmesh gate; workers only read the
/// captured frame and write their own agent. No component callbacks run here.
/// </summary>
internal sealed class NavigationSimulation
{
	internal readonly object Gate;
	private readonly NavMeshGraph mesh;
	private readonly MeshQuery planningQuery;
	private readonly List<SimulationAgent> agents = new();
	private readonly Dictionary<(int, int), List<int>> grid = new();
	private readonly Stack<List<int>> buckets = new();
	private FrameAgent[] frame = [];
	private float cellSize;
	internal readonly Vector3 PlacementExtents;
	internal long Revision;
	private long update;
	private float stepDelta;
	private readonly Action<int> stepWorker;
	internal int Count { get { lock ( Gate ) return agents.Count; } }

	internal NavigationSimulation( NavMeshGraph mesh, object gate, float radius, float height )
	{
		this.mesh = mesh;
		stepWorker = index => Step( index, stepDelta );
		planningQuery = new MeshQuery( mesh );
		Gate = gate;
		PlacementExtents = new Vector3( radius * 2.1f, height * 1.51f, radius * 2.1f );
	}

	internal SimulationAgent Add( Vector3 position, SimulationSettings settings )
	{
		lock ( Gate )
		{
			var agent = new SimulationAgent( this, mesh, position, settings );
			agents.Add( agent );
			return agent;
		}
	}

	internal void Remove( SimulationAgent agent )
	{
		lock ( Gate ) { agents.Remove( agent ); agent.Removed = true; }
	}

	internal void Invalidate()
	{
		lock ( Gate )
		{
			foreach ( var agent in agents )
			{
				agent.Removed = true;
				agent.Velocity = agent.WishVelocity = default;
				agent.Link = null;
			}
			agents.Clear();
		}
	}

	internal void Update( float delta )
	{
		if ( !float.IsFinite( delta ) || delta <= 0 ) return;
		lock ( Gate )
		{
			update++;
			// Bound local movement so MoveAlongSurface cannot cross an arbitrary
			// number of polygons after a paused frame.
			int steps = Math.Clamp( (int)MathF.Ceiling( delta / 0.05f ), 1, 8 );
			float dt = MathF.Min( delta, 0.4f ) / steps;
			stepDelta = dt;
			for ( int step = 0; step < steps; step++ )
			{
				foreach ( var agent in agents ) Prepare( agent );
				CaptureFrame();
				if ( agents.Count < 64 )
				{
					for ( int i = 0; i < agents.Count; i++ ) Step( i, dt );
				}
				else
				{
					Parallel.For( 0, agents.Count, stepWorker );
				}
			}
		}
	}

	private void Prepare( SimulationAgent agent )
	{
		if ( agent.Link is not null ) return;
		var query = agent.Query;
		var settings = agent.Options;
		if ( agent.Path.Count == 0 || !query.IsValidPolyRef( agent.Path[0], settings.Filter ) )
		{
			var extents = Vector3.Max( PlacementExtents, new Vector3( settings.Radius * 2.1f, settings.Height * 1.51f, settings.Radius * 2.1f ) );
			var status = query.FindNearestPoly( agent.Position, extents, settings.Filter, out var poly, out var point, out _ );
			if ( status.Failed() || poly == 0 )
			{
				agent.Path.Clear();
				agent.CornerCount = 0;
				agent.NeedsPath = agent.Target.HasValue;
				agent.Velocity = agent.WishVelocity = default;
				return;
			}
			agent.Position = point;
			agent.Path.Clear();
			agent.Path.Add( poly );
			agent.NeedsPath = agent.Target.HasValue;
		}
		if ( agent.PathRevision != Revision )
		{
			if ( agent.Target.HasValue )
			{
				agent.NeedsPath |= agent.Partial;
				foreach ( var polygon in agent.Path )
					if ( !query.IsValidPolyRef( polygon, settings.Filter ) ) { agent.NeedsPath = true; break; }
			}
			agent.PathRevision = Revision;
		}
		if ( agent.NeedsPath && agent.Target is Vector3 target )
		{
			var status = query.FindNearestPoly( target, new Vector3( 256 ), settings.Filter, out var poly, out var point, out _ );
			if ( status.Failed() || poly == 0 ) return;
			long start = agent.Path[0];
			status = planningQuery.FindPath( start, poly, agent.Position, point, settings.Filter, agent.Path );
			if ( status.Failed() || agent.Path.Count == 0 ) { agent.Path.Clear(); agent.Path.Add( start ); return; }
			agent.Partial = agent.Path[^1] != poly;
			if ( agent.Partial ) query.ClosestPointOnPoly( agent.Path[^1], point, out point, out _ );
			agent.PathTarget = point;
			agent.NeedsPath = false;
		}
	}

	private void CaptureFrame()
	{
		if ( frame.Length < agents.Count ) Array.Resize( ref frame, Math.Max( agents.Count, frame.Length * 2 ) );
		foreach ( var bucket in grid.Values ) { bucket.Clear(); buckets.Push( bucket ); }
		grid.Clear();
		cellSize = 1;
		for ( int i = 0; i < agents.Count; i++ )
		{
			var agent = agents[i];
			frame[i] = new FrameAgent( agent.Position, agent.Velocity, agent.Options.Radius, agent.Options.Height, agent.Link is null );
			cellSize = MathF.Max( cellSize, agent.Options.Radius * 4 + agent.Options.MaxSpeed );
		}
		for ( int i = 0; i < agents.Count; i++ )
		{
			if ( !frame[i].Walking ) continue;
			var key = Cell( frame[i].Position );
			if ( !grid.TryGetValue( key, out var bucket ) )
			{
				bucket = buckets.Count > 0 ? buckets.Pop() : new List<int>();
				grid.Add( key, bucket );
			}
			bucket.Add( i );
		}
	}

	private (int x, int z) Cell( Vector3 position ) => ((int)MathF.Floor( position.x / cellSize ), (int)MathF.Floor( position.z / cellSize ));

	private void Step( int index, float dt )
	{
		var agent = agents[index];
		var settings = agent.Options;
		if ( agent.Link is SimulationLink link )
		{
			// Publish every link entry for at least one complete update, including
			// links shorter than a movement substep, so component events see it.
			if ( agent.LinkStartedUpdate == update ) return;
			if ( settings.AutoTraverseLinks && settings.MaxSpeed > 0 )
			{
				var offset = link.End - agent.Position;
				float distance = offset.Length;
				if ( distance <= settings.MaxSpeed * dt ) agent.CompleteLinkCore();
				else agent.Position += offset / distance * settings.MaxSpeed * dt;
			}
			return;
		}
		if ( agent.Path.Count == 0 || !agent.Target.HasValue || agent.NeedsPath ) return;

		// End the funnel at the first link entrance. A vertical link has zero
		// horizontal width and can otherwise disappear from a 2D funnel.
		Vector3 end = agent.PathTarget;
		int count = agent.Path.Count;
		long linkRef = 0;
		Vector3 linkEnd = default;
		for ( int i = 1; i < count; i++ )
		{
			if ( mesh.GetTileAndPolyByRef( agent.Path[i], out _, out var poly ).Failed() ) { agent.NeedsPath = true; return; }
			if ( poly.type != PolyTypes.POLYTYPE_OFFMESH_CONNECTION ) continue;
			if ( mesh.GetOffMeshConnectionPolyEndPoints( agent.Path[i - 1], agent.Path[i], ref end, ref linkEnd ).Failed() ) { agent.NeedsPath = true; return; }
			linkRef = agent.Path[i];
			count = i;
			break;
		}
		if ( linkRef != 0 && Geometry.DistanceBetween2DSqr( agent.Position, end ) <= MathF.Max( 0.1f, settings.Radius ) * MathF.Max( 0.1f, settings.Radius ) )
		{
			agent.Link = new SimulationLink( linkRef, end, linkEnd, agent.Position );
			agent.LinkStartedUpdate = update;
			agent.Path.RemoveRange( 0, count + 1 );
			agent.Velocity = agent.WishVelocity = default;
			return;
		}
		var status = agent.Query.FindStraightPath( agent.Position, end, agent.Path, count, agent.Corners, out agent.CornerCount, agent.Corners.Length, 0 );
		if ( status.Failed() ) { agent.NeedsPath = true; return; }
		Vector3 direction = default;
		for ( int i = 0; i < agent.CornerCount; i++ )
		{
			direction = agent.Corners[i].pos - agent.Position;
			direction.y = 0;
			if ( direction.LengthSquared > 0.01f ) break;
		}
		float distanceToEnd = Geometry.DistanceBetween2D( agent.Position, end );
		if ( linkRef == 0 && distanceToEnd < 0.1f )
		{
			// A partial route must retain its destination so tile changes can resume it.
			if ( !agent.Partial ) agent.Target = null;
			agent.Velocity = agent.WishVelocity = default;
			return;
		}
		float speed = MathF.Min( settings.MaxSpeed, MathF.Sqrt( 2 * settings.Acceleration * distanceToEnd ) );
		var wish = direction.Normal * speed;
		agent.WishVelocity = wish;
		var avoidance = Avoid( index, wish, dt );
		var change = avoidance - agent.Velocity;
		float maxChange = settings.Acceleration * dt;
		if ( change.Length > maxChange ) change = change.Normal * maxChange;
		var velocity = agent.Velocity + change;
		var displacement = velocity * dt;
		if ( displacement.Length > distanceToEnd && linkRef == 0 ) displacement = displacement.Normal * distanceToEnd;
		var old = agent.Position;
		Span<long> visited = stackalloc long[32];
		status = agent.Query.MoveAlongSurface( agent.Path[0], old, old + displacement, settings.Filter, out var position, visited, out int visitedCount, visited.Length );
		if ( status.Failed() ) { agent.NeedsPath = true; return; }
		// Keep the corridor suffix after the furthest common polygon.
		int commonPath = -1, commonVisited = -1;
		for ( int p = agent.Path.Count - 1; p >= 0 && commonPath < 0; p-- )
			for ( int v = visitedCount - 1; v >= 0; v-- )
				if ( agent.Path[p] == visited[v] ) { commonPath = p; commonVisited = v; break; }
		if ( commonPath >= 0 )
		{
			agent.Path.RemoveRange( 0, commonPath );
			for ( int v = commonVisited + 1; v < visitedCount; v++ ) agent.Path.Insert( 0, visited[v] );
			// Sliding across a seam or avoiding an agent can leave the planned
			// corridor. Replan from that polygon instead of steering backwards
			// through the old entrance, which can create a loop at tile corners.
			agent.NeedsPath = commonVisited < visitedCount - 1;
		}
		if ( agent.Query.GetPolyHeight( agent.Path[0], position, out float height ).Succeeded() ) position.y = height;
		agent.Position = position;
		agent.Velocity = (position - old) / dt;
	}

	private Vector3 Avoid( int index, Vector3 wish, float dt )
	{
		var self = frame[index];
		var settings = agents[index].Options;
		var cell = Cell( self.Position );
		var correction = Vector3.Zero;
		for ( int x = cell.x - 1; x <= cell.x + 1; x++ )
			for ( int z = cell.z - 1; z <= cell.z + 1; z++ )
			{
				if ( !grid.TryGetValue( (x, z), out var bucket ) ) continue;
				foreach ( int otherIndex in bucket )
				{
					if ( index == otherIndex ) continue;
					var other = frame[otherIndex];
					if ( self.Position.y + self.Height <= other.Position.y || other.Position.y + other.Height <= self.Position.y ) continue;
					var offset = self.Position - other.Position;
					offset.y = 0;
					float radius = self.Radius + other.Radius;
					float distance = offset.Length;
					var normal = distance > 0.001f ? offset / distance : new Vector3( index < otherIndex ? -1 : 1, 0, 0 );
					if ( distance < radius ) correction += normal * ((radius - distance) / MathF.Max( dt, 0.01f )) * 0.5f;
					else
					{
						var relative = wish - other.Velocity;
						float closing = -Vector3.Dot( relative, normal );
						if ( closing > 0 && distance - radius < closing * 0.75f )
						{
							// Consistent passing side breaks head-on symmetry.
							var side = new Vector3( -normal.z, 0, normal.x );
							correction += (normal + side) * (closing - (distance - radius) / 0.75f) * 0.5f;
						}
						if ( distance < radius * 2 ) correction += normal * (1 - (distance - radius) / MathF.Max( radius, 0.01f )) * settings.Separation * settings.MaxSpeed;
					}
				}
			}
		var result = wish + correction;
		return result.Length > settings.MaxSpeed ? result.Normal * settings.MaxSpeed : result;
	}

	private readonly record struct FrameAgent( Vector3 Position, Vector3 Velocity, float Radius, float Height, bool Walking );
}

internal readonly record struct SimulationSettings( float Radius, float Height, float MaxSpeed, float Acceleration, float Separation, bool AutoTraverseLinks, TraversalFilter Filter );
internal readonly record struct SimulationLink( long Polygon, Vector3 Start, Vector3 End, Vector3 Initial );
internal readonly record struct SimulationState( Vector3 Position, Vector3 Velocity, Vector3 WishVelocity, Vector3? Target, bool Navigating, SimulationLink? Link );

internal sealed class SimulationAgent
{
	internal readonly NavigationSimulation Owner;
	internal readonly MeshQuery Query;
	internal SimulationSettings Options;
	internal Vector3 Position, Velocity, WishVelocity, PathTarget;
	internal Vector3? Target;
	internal SimulationLink? Link;
	internal readonly List<long> Path = new();
	internal readonly StraightPath[] Corners = new StraightPath[16];
	internal int CornerCount;
	internal bool NeedsPath, Partial, Removed;
	internal long PathRevision = -1;
	internal long LinkStartedUpdate;

	internal SimulationAgent( NavigationSimulation owner, NavMeshGraph mesh, Vector3 position, SimulationSettings settings )
	{
		Owner = owner;
		Query = new MeshQuery( mesh );
		Position = position;
		Options = settings;
	}

	internal SimulationState State
	{
		get { lock ( Owner.Gate ) return new( Position, Velocity, WishVelocity, Target, !Removed && Target.HasValue && !NeedsPath && Path.Count > 0, Link ); }
	}

	internal void MoveTo( Vector3 target )
	{
		lock ( Owner.Gate )
		{
			if ( Removed || Target is Vector3 previous && previous.AlmostEqual( target, 1 ) ) return;
			Target = target;
			NeedsPath = true;
		}
	}

	internal void SetPosition( Vector3 position )
	{
		lock ( Owner.Gate )
		{
			Position = position;
			if ( Link is null ) { Path.Clear(); NeedsPath = Target.HasValue; }
		}
	}

	internal void Stop()
	{
		lock ( Owner.Gate )
		{
			if ( Link is not null ) Path.Clear();
			Target = null;
			Link = null;
			NeedsPath = false;
			Velocity = WishVelocity = default;
			CornerCount = 0;
		}
	}

	internal void CompleteLink()
	{
		lock ( Owner.Gate ) CompleteLinkCore();
	}

	internal void CompleteLinkCore()
	{
		if ( Link is not SimulationLink link ) return;
		Position = link.End;
		Link = null;
		Velocity = WishVelocity = default;
	}
}
