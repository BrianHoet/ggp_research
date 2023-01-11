#pragma once
#include <vector>
#include <iostream>
#include "framework/EliteMath/EMath.h"
#include "framework\EliteAI\EliteGraphs\ENavGraph.h"
#include "framework\EliteAI\EliteGraphs\EliteGraphAlgorithms\EAStar.h"

namespace Elite
{
	class NavMeshPathfinding
	{
	public:
		static std::vector<Vector2> FindPath(Vector2 startPos, Vector2 endPos, NavGraph* pNavGraph, std::vector<Vector2>& debugNodePositions, std::vector<Portal>& debugPortals)
		{
			//Create the path to return
			std::vector<Vector2> finalPath{};

			//Get the start and endTriangle
			auto startTriangle = pNavGraph->GetNavMeshPolygon()->GetTriangleFromPosition(startPos);
			auto endTriangle = pNavGraph->GetNavMeshPolygon()->GetTriangleFromPosition(endPos);

			if (!startTriangle || !endTriangle) return finalPath;

			if (startTriangle == endTriangle) finalPath.push_back(endPos);
			//We have valid start/end triangles and they are not the same
			//=> Start looking for a path
			//Copy the graph
			auto clonedGraph = pNavGraph->Clone();

			//Create extra node for the Start Node (Agent's position
			NavGraphNode* startNode = new NavGraphNode{ clonedGraph->GetNextFreeNodeIndex(), -1, startPos };
			int startNodeIndex{ clonedGraph->AddNode(startNode) };

			for (auto Index : startTriangle->metaData.IndexLines)
			{
				auto nodeIndex{ pNavGraph->GetNodeIdxFromLineIdx(Index) };
				if (nodeIndex != invalid_node_index)
				{
					auto node{ clonedGraph->GetNode(nodeIndex) };

					clonedGraph->AddConnection(new GraphConnection2D{ startNodeIndex, nodeIndex, Distance(startPos, node->GetPosition()) });
				}
			}
			//Create extra node for the endNode
			NavGraphNode* endNode = new NavGraphNode{ clonedGraph->GetNextFreeNodeIndex(), -1, endPos };
			int endNodeIndex{ clonedGraph->AddNode(endNode) };

			for (auto Index : endTriangle->metaData.IndexLines)
			{
				auto nodeIndex{ pNavGraph->GetNodeIdxFromLineIdx(Index) };
				if (nodeIndex != invalid_node_index)
				{
					auto node{ clonedGraph->GetNode(nodeIndex) };

					clonedGraph->AddConnection(new GraphConnection2D{ endNodeIndex, nodeIndex, Distance(endPos, node->GetPosition()) });
				}
			}

			//Run A star on new graph
			auto aStar = AStar<NavGraphNode, GraphConnection2D>(&(*clonedGraph), Elite::HeuristicFunctions::Euclidean);
			auto path = aStar.FindPath(startNode, endNode);

			//OPTIONAL BUT ADVICED: Debug Visualisation
			for (const auto i : path)
			{
				finalPath.push_back(i->GetPosition());
				debugNodePositions.push_back(i->GetPosition());
			}

			//Run optimiser on new graph, MAKE SURE the A star path is working properly before starting this section and uncommenting this!!!
			std::vector<Portal> portals = SSFA::FindPortals(path, pNavGraph->GetNavMeshPolygon());
			finalPath = SSFA::OptimizePortals(portals);

			debugPortals.clear();
			for(const auto i: portals)
			{
				debugPortals.push_back(i);
			}

			return finalPath;
		}
	};
}
