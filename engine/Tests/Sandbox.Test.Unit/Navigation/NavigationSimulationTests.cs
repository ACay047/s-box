using System;
using System.Collections.Generic;
using Sandbox.Navigation;
using Sandbox.Navigation.Pathfinding;

namespace NavigationTests;

[TestClass]
public class NavigationSimulationTests
{
	[TestMethod]
	public void SpanConnectivityAndSearchQueueMatchReference() => NavigationAlgorithmChecks.Run();

	[TestMethod]
	public void BoundsSearchAndTileReuse() => NavigationAlgorithmChecks.BoundsAndSearch();

	[TestMethod]
	public void LayerRegionsSupportMoreRowSpansThanColumns()
	{
		using var field = new Sandbox.Navigation.Generation.Heightfield( 8, 8, Vector3.Zero, new Vector3( 8, 512, 8 ), 1, 1 );
		for ( int z = 0; z < 8; z++ )
			for ( int x = 0; x < 8; x += 2 )
				for ( int y = 0; y < 8; y++ ) field.AddOrMergeSpan( x, z, (ushort)(y * 32), (ushort)(y * 32 + 1), 1, 0 );
		using var compact = field.BuildCompactHeightfield( 8, 4 );
		Assert.IsTrue( Sandbox.Navigation.Generation.RegionBuilder.BuildLayerRegions( compact, 0, 0, new(), new() ) );
		Assert.IsTrue( compact.MaxRegions >= 32 );
		using var copy = compact.Copy();
		Assert.AreEqual( compact.BMax, copy.BMax, "Copying a cached field must not expand its bounds" );
	}
	[TestMethod]
	public void WarmSerialSimulationDoesNotAllocate()
	{
		var mesh = SyntheticNavMesh.Create();
		var simulation = new NavigationSimulation( mesh, new object(), 8, 32 );
		var agent = simulation.Add( new Vector3( 100, 1, 100 ), Settings( mesh ) );
		agent.MoveTo( new Vector3( 500, 1, 100 ) );
		for ( int i = 0; i < 32; i++ ) simulation.Update( 0.001f );
		long before = GC.GetAllocatedBytesForCurrentThread();
		for ( int i = 0; i < 128; i++ ) simulation.Update( 0.001f );
		long allocated = GC.GetAllocatedBytesForCurrentThread() - before;
		Assert.IsTrue( allocated <= 256, $"Warmed serial updates allocated {allocated} bytes" );
		Assert.IsTrue( agent.State.Position.x > 100 );
	}

	[TestMethod]
	public void PartialRouteResumesWhenAConnectionAppears()
	{
		var mesh = SyntheticNavMesh.Create( upperFloor: true );
		var simulation = new NavigationSimulation( mesh, new object(), 8, 32 );
		var agent = simulation.Add( new Vector3( 100, 1, 320 ), Settings( mesh ) );
		var target = new Vector3( 500, 201, 320 );
		agent.MoveTo( target );
		for ( int i = 0; i < 500; i++ ) simulation.Update( 0.02f );
		Assert.IsTrue( agent.Partial );
		Assert.AreEqual( (Vector3?)target, agent.State.Target );
		var connected = SyntheticNavMesh.Create( upperFloor: true, link: true );
		Assert.IsTrue( mesh.UpdateTile( connected.GetTile( 0 ).data, 0 ).Succeeded() );
		simulation.Revision++;
		for ( int i = 0; i < 700; i++ ) simulation.Update( 0.02f );
		Assert.IsTrue( agent.State.Position.Distance( target ) < 1 );
		Assert.IsNull( agent.State.Target );
	}

	[TestMethod]
	public void AgentWaitsWhenItsTileDisappearsAndResumesWhenRestored()
	{
		var mesh = SyntheticNavMesh.Create();
		var simulation = new NavigationSimulation( mesh, new object(), 8, 32 );
		var agent = simulation.Add( new Vector3( 100, 1, 100 ), Settings( mesh ) );
		var target = new Vector3( 500, 1, 100 );
		agent.MoveTo( target );
		simulation.Update( 0.02f );
		var data = mesh.GetTile( 0 ).data;
		mesh.RemoveTile( mesh.GetTileRef( mesh.GetTile( 0 ) ) );
		simulation.Revision++;
		var position = agent.State.Position;
		simulation.Update( 0.02f );
		Assert.AreEqual( 0, agent.Path.Count );
		Assert.IsFalse( agent.State.Navigating );
		Assert.AreEqual( position, agent.State.Position );
		Assert.AreEqual( (Vector3?)target, agent.State.Target );
		mesh.AddTile( data, 0, 0, out _ );
		simulation.Revision++;
		for ( int i = 0; i < 400; i++ ) simulation.Update( 0.02f );
		Assert.IsTrue( agent.State.Position.Distance( target ) < 1 );
	}

	[TestMethod]
	public void StoppingOnALinkReprojectsOntoTheCurrentFloor()
	{
		var mesh = SyntheticNavMesh.Create( upperFloor: true, link: true );
		var simulation = new NavigationSimulation( mesh, new object(), 8, 32 );
		var agent = simulation.Add( new Vector3( 320, 1, 320 ), Settings( mesh, false ) );
		agent.MoveTo( new Vector3( 500, 201, 320 ) );
		simulation.Update( 0.02f );
		Assert.IsTrue( agent.State.Link.HasValue );
		agent.Stop();
		simulation.Update( 0.02f );
		Assert.IsNull( agent.State.Link );
		Assert.IsTrue( agent.Query.GetPolyHeight( agent.Path[0], agent.Position, out float height ).Succeeded() );
		Assert.AreEqual( 1f, height );
		agent.MoveTo( new Vector3( 100, 1, 320 ) );
		for ( int i = 0; i < 300; i++ ) simulation.Update( 0.02f );
		Assert.IsTrue( agent.State.Position.Distance( new Vector3( 100, 1, 320 ) ) < 1 );
	}

	static SimulationSettings Settings( NavMeshGraph mesh, bool automatic = true ) => new( 8, 32, 120, 240, 0.25f, automatic, TraversalFilter.Unrestricted );

	[TestMethod]
	public void AirborneAgentRecoversAndKeepsTarget_10230()
	{
		var mesh = SyntheticNavMesh.Create();
		var simulation = new NavigationSimulation( mesh, new object(), 8, 32 );
		var agent = simulation.Add( new Vector3( 100, 500, 100 ), Settings( mesh ) );
		agent.MoveTo( new Vector3( 500, 1, 100 ) );
		simulation.Update( 0.02f );
		Assert.AreEqual( 500f, agent.State.Position.y, "An unsuccessful placement must not reset position to the origin" );
		agent.SetPosition( new Vector3( 100, 1, 100 ) );
		for ( int i = 0; i < 300; i++ ) simulation.Update( 0.02f );
		Assert.IsTrue( agent.State.Position.Distance( new Vector3( 500, 1, 100 ) ) < 1 );
	}

	[TestMethod]
	public void VerticalLinkTraversesInBothDirections_10146()
	{
		var mesh = SyntheticNavMesh.Create( upperFloor: true, link: true );
		var simulation = new NavigationSimulation( mesh, new object(), 8, 32 );
		var agent = simulation.Add( new Vector3( 100, 1, 320 ), Settings( mesh ) );
		foreach ( float height in new[] { 201f, 1f } )
		{
			agent.MoveTo( new Vector3( 500, height, 320 ) );
			bool entered = false;
			for ( int i = 0; i < 500; i++ ) { simulation.Update( 0.02f ); entered |= agent.State.Link.HasValue; }
			Assert.IsTrue( entered );
			Assert.IsTrue( agent.State.Position.Distance( new Vector3( 500, height, 320 ) ) < 1, agent.State.ToString() );
		}
	}

	[TestMethod]
	public void ManualLinkWaitsForCompletion_10146()
	{
		var mesh = SyntheticNavMesh.Create( upperFloor: true, link: true );
		var simulation = new NavigationSimulation( mesh, new object(), 8, 32 );
		var agent = simulation.Add( new Vector3( 100, 1, 320 ), Settings( mesh, false ) );
		agent.MoveTo( new Vector3( 500, 201, 320 ) );
		for ( int i = 0; i < 300; i++ ) simulation.Update( 0.02f );
		Assert.IsTrue( agent.State.Link.HasValue );
		Assert.AreEqual( 1f, agent.State.Position.y, 0.01f );
		agent.CompleteLink();
		for ( int i = 0; i < 300; i++ ) simulation.Update( 0.02f );
		Assert.IsFalse( agent.State.Link.HasValue );
		Assert.IsTrue( agent.State.Position.Distance( new Vector3( 500, 201, 320 ) ) < 1 );
	}

	[TestMethod]
	public void ShortLinkEntryIsPublishedBeforeCompletion()
	{
		var mesh = SyntheticNavMesh.Create( upperFloor: true, link: true );
		var simulation = new NavigationSimulation( mesh, new object(), 8, 32 );
		var agent = simulation.Add( new Vector3( 320, 1, 320 ), Settings( mesh ) with { MaxSpeed = 100000 } );
		agent.MoveTo( new Vector3( 500, 201, 320 ) );
		simulation.Update( 0.4f );
		Assert.IsTrue( agent.State.Link.HasValue, "Substeps must not consume the entry before component callbacks can observe it" );
		simulation.Update( 0.02f );
		Assert.IsFalse( agent.State.Link.HasValue );
	}

	[TestMethod]
	public void HeadOnAgentsPassAndReachTheirTargets()
	{
		var mesh = SyntheticNavMesh.Create();
		var simulation = new NavigationSimulation( mesh, new object(), 8, 32 );
		var start = new Vector3( 100, 1, 320 );
		var end = new Vector3( 500, 1, 320 );
		var left = simulation.Add( start, Settings( mesh ) );
		var right = simulation.Add( end, Settings( mesh ) );
		left.MoveTo( end );
		right.MoveTo( start );
		float minimumDistance = float.MaxValue;
		for ( int i = 0; i < 500; i++ )
		{
			simulation.Update( 0.02f );
			minimumDistance = MathF.Min( minimumDistance, left.State.Position.Distance( right.State.Position ) );
		}
		Assert.IsTrue( minimumDistance >= 16, $"Agents overlapped: distance {minimumDistance}" );
		Assert.IsTrue( left.State.Position.Distance( end ) < 1 );
		Assert.IsTrue( right.State.Position.Distance( start ) < 1 );
	}

	[TestMethod]
	public void CommandsAndSnapshotsAreSafeDuringParallelSimulation()
	{
		var mesh = SyntheticNavMesh.Create();
		var simulation = new NavigationSimulation( mesh, new object(), 8, 32 );
		var agents = Enumerable.Range( 0, 100 ).Select( i => simulation.Add( new Vector3( 50 + i % 10 * 50, 1, 50 + i / 10 * 50 ), Settings( mesh ) ) ).ToArray();
		Parallel.Invoke(
			() => { for ( int i = 0; i < 100; i++ ) simulation.Update( 0.02f ); },
			() => { foreach ( var agent in agents ) { agent.MoveTo( new Vector3( 320, 1, 320 ) ); Assert.IsTrue( float.IsFinite( agent.State.Position.x ) ); agent.Stop(); } } );
		Assert.AreEqual( 100, simulation.Count );
	}
}
