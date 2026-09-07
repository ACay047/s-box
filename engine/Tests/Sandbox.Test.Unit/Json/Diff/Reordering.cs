using System.Collections.Generic;
using System.Linq;
using System.Text.Json.Nodes;

namespace JsonTests.Diff;

[TestClass]
public class ReorderingTest
{
	private static HashSet<Json.TrackedObjectDefinition> Definitions() =>
	[
		Json.TrackedObjectDefinition.CreatePresenceBasedDefinition( "Root", ["root"], allowedAsRoot: true ),
		Json.TrackedObjectDefinition.CreatePresenceBasedDefinition( "Item", ["id"], "id", "Root" )
	];

	private static JsonObject Tree( params int[][] arrays )
	{
		var root = new JsonObject { ["root"] = true };
		for ( var i = 0; i < arrays.Length; i++ )
			root[$"items{i}"] = new JsonArray( arrays[i].Select( id => (JsonNode)new JsonObject { ["id"] = id } ).ToArray() );
		return root;
	}

	[TestMethod]
	[DataRow( false )]
	[DataRow( true )]
	public void ReversedDependenciesAcrossMultipleArrays( bool additions )
	{
		var first = Enumerable.Range( 0, 1000 ).ToArray();
		var second = Enumerable.Range( 1000, 1000 ).ToArray();
		var source = additions ? Tree( [], [] ) : Tree( first, second );
		var target = Tree( first.Reverse().ToArray(), second.Reverse().ToArray() );
		var definitions = Definitions();
		var patch = Json.CalculateDifferences( source, target, definitions );
		patch.AddedObjects.Reverse();
		patch.MovedObjects.Reverse();

		Assert.IsTrue( JsonNode.DeepEquals( target, Json.ApplyPatch( source, patch, definitions ) ) );
		Assert.IsTrue( JsonNode.DeepEquals( target, Json.ApplyPatch( source, patch, definitions ) ) );
	}

	[TestMethod]
	public void CyclicPredecessorsUseBoundedFallback()
	{
		var patch = new Json.Patch();
		foreach ( var (id, previous) in new[] { ("1", "2"), ("2", "1") } )
		{
			patch.MovedObjects.Add( new Json.MovedObject
			{
				Id = new() { Type = "Item", IdValue = id },
				NewParent = new() { Type = "Root" },
				NewContainerProperty = "items0",
				IsNewContainerArray = true,
				NewPreviousElement = new Json.ObjectIdentifier { Type = "Item", IdValue = previous }
			} );
		}

		var result = Json.ApplyPatch( Tree( [1, 2, 3] ), patch, Definitions() );
		Assert.IsTrue( JsonNode.DeepEquals( Tree( [3, 1, 2] ), result ) );
	}

	[TestMethod]
	public void PartialPatchWithMissingPredecessors()
	{
		var definitions = Definitions();
		var patch = Json.CalculateDifferences( Tree( [1, 2, 3] ), Tree( [1, 4, 2, 5, 3] ), definitions );
		// Both added objects lose their anchors. Preserve the bounded fallback order.
		var result = Json.ApplyPatch( Tree( [3] ), patch, definitions );
		Assert.IsTrue( JsonNode.DeepEquals( Tree( [5, 3, 4] ), result ) );
	}
}
