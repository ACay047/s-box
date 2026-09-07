using System;
using System.Linq;
using System.Text.Json.Nodes;

namespace SceneTests.Prefab;

public partial class InstancesTest
{
	[TestMethod]
	[DataRow( false )]
	[DataRow( true )]
	public void PrefabCacheLoadsOwnBlobsAndKeepsSnapshotSelfContained( bool ambientCapture )
	{
		const string path = "___prefab_cache_blobs.prefab";
		var scene = new Scene();
		using var scope = scene.Push();
		var source = CreateMeshBlock( "BlobMesh" );
		var sourceMesh = source.Components.Get<MeshComponent>().Mesh;
		var vertices = sourceMesh.VertexHandles.Count();
		var faces = sourceMesh.FaceHandles.Count();
		Assert.IsTrue( vertices > 0 && faces > 0 );

		JsonObject root;
		byte[] data;
		using ( var capture = BlobDataSerializer.Capture() )
		{
			root = source.Serialize();
			data = capture.ToByteArray();
		}
		Assert.IsTrue( root.ToJsonString().Contains( "$blob" ) );
		Assert.IsNotNull( data );

		var file = new PrefabFile { RootObject = root, BinaryData = data };
		file.RegisterWeakResourceId( path );
		file.Register( path );
		try
		{
			PrefabCacheScene cached;
			using ( var ambient = ambientCapture ? BlobDataSerializer.Capture() : null )
			{
				cached = (PrefabCacheScene)GameObject.GetPrefab( path );
				Assert.AreEqual( vertices, cached.Components.Get<MeshComponent>().Mesh.VertexHandles.Count() );
				Assert.IsFalse( cached.FullPrefabInstanceJson.ToJsonString().Contains( "$blob" ) );
			}

			// Both the resource load context and any unrelated ambient capture are now gone.
			var instance = cached.Clone();
			var mesh = instance.Components.Get<MeshComponent>().Mesh;
			Assert.AreEqual( vertices, mesh.VertexHandles.Count() );
			Assert.AreEqual( faces, mesh.FaceHandles.Count() );
		}
		finally
		{
			Game.Resources.Unregister( file );
		}
	}
}
