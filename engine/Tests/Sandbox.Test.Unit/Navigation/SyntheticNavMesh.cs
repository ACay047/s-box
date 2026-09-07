using System;
using System.Collections.Generic;
using Sandbox.Navigation.Generation;
using Sandbox.Navigation.Pathfinding;

namespace NavigationTests;

internal static class SyntheticNavMesh
{
	internal static NavMeshGraph Create( bool upperFloor = false, bool link = false, bool obstacles = false, bool rasterized = false )
	{
		using var field = new Heightfield( 64, 64, Vector3.Zero, new Vector3( 640, 512, 640 ), 10, 1 );
		var vertices = new List<Vector3>();
		var indices = new List<int>();
		var areas = new List<int>();
		for ( int z = 0; z < 64; z++ )
			for ( int x = 0; x < 64; x++ )
			{
				if ( obstacles && x % 12 == 6 && z > 8 && z < 56 ) continue;
				if ( rasterized )
				{
					int start = vertices.Count;
					vertices.AddRange( new Vector3[] { new( x * 10, 0, z * 10 ), new( x * 10, 0, (z + 1) * 10 ), new( (x + 1) * 10, 0, (z + 1) * 10 ), new( (x + 1) * 10, 0, z * 10 ) } );
					indices.AddRange( new[] { start, start + 1, start + 2, start, start + 2, start + 3 } );
					areas.AddRange( new[] { Constants.WALKABLE_AREA, Constants.WALKABLE_AREA } );
					continue;
				}
				field.AddOrMergeSpan( x, z, 0, 1, Constants.WALKABLE_AREA, 0 );
				if ( upperFloor ) field.AddOrMergeSpan( x, z, 200, 201, Constants.WALKABLE_AREA, 0 );
			}
		if ( rasterized ) Rasterization.RasterizeTriangles( vertices.ToArray(), indices.ToArray(), areas.ToArray(), field, 0 );
		using var compact = field.BuildCompactHeightfield( 32, 8 );
		RegionBuilder.BuildLayerRegions( compact, 0, 0, new(), new() );
		var contours = new ContourBuilder.ContourBuilderContext();
		ContourBuilder.BuildContours( compact, 1, 8, contours );
		using var polygons = PolyMeshBuilder.BuildPolyMesh( contours.ContourSet, 6, new() );
		var parameters = new MeshBuildParameters
		{
			pmesh = polygons,
			bmin = polygons.BMin,
			bmax = polygons.BMax,
			cs = 10,
			ch = 1,
			walkableHeight = 32,
			walkableRadius = 8,
			walkableClimb = 8,
			buildBvTree = true,
			offMeshConVerts = link ? new Vector3[] { new( 320, 1, 320 ), new( 320, 201, 320 ) } : [],
			offMeshConRad = link ? new float[] { 24 } : [],
			offMeshConAreas = link ? new int[] { Constants.WALKABLE_AREA } : [],
			offMeshConBidirectional = link ? new bool[] { true } : [],
			offMeshConUserData = link ? new object[] { null } : [],
			offMeshConCount = link ? 1 : 0
		};
		var data = MeshBuilder.CreateNavMeshData( parameters );
		var mesh = new NavMeshGraph();
		mesh.Init( new MeshParameters { orig = Vector3.Zero, tileWidth = 640, tileHeight = 640, maxTiles = 4 }, 6 );
		mesh.AddTile( data, 0, 0, out _ );
		return mesh;
	}
}
