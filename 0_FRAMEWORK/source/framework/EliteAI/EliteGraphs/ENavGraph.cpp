#include "stdafx.h"
#include "ENavGraph.h"
#include "framework\EliteAI\EliteGraphs\EliteGraphAlgorithms\EAStar.h"

using namespace Elite;

Elite::NavGraph::NavGraph(const Polygon& contourMesh, float playerRadius = 1.0f) :
	Graph2D(false),
	m_pNavMeshPolygon(nullptr)
{
	//Create the navigation mesh (polygon of navigatable area= Contour - Static Shapes)
	m_pNavMeshPolygon = new Polygon(contourMesh); // Create copy on heap

	//Get all shapes from all static rigidbodies with NavigationCollider flag
	auto vShapes = PHYSICSWORLD->GetAllStaticShapesInWorld(PhysicsFlags::NavigationCollider);

	//Store all children
	for (auto shape : vShapes)
	{
		shape.ExpandShape(playerRadius);
		m_pNavMeshPolygon->AddChild(shape);
	}

	//Triangulate
	m_pNavMeshPolygon->Triangulate();

	//Create the actual graph (nodes & connections) from the navigation mesh
	CreateNavigationGraph();
}

Elite::NavGraph::~NavGraph()
{
	delete m_pNavMeshPolygon; 
	m_pNavMeshPolygon = nullptr;
}

int Elite::NavGraph::GetNodeIdxFromLineIdx(int lineIdx) const
{
	auto nodeIt = std::find_if(m_Nodes.begin(), m_Nodes.end(), [lineIdx](const NavGraphNode* n) { return n->GetLineIndex() == lineIdx; });
	if (nodeIt != m_Nodes.end())
	{
		return (*nodeIt)->GetIndex();
	}

	return invalid_node_index;
}

Elite::Polygon* Elite::NavGraph::GetNavMeshPolygon() const
{
	return m_pNavMeshPolygon;
}

void Elite::NavGraph::CreateNavigationGraph()
{
	//1. Go over all the edges of the navigationmesh and create nodes
	for (const auto& lines : m_pNavMeshPolygon->GetLines())
	{
		if (m_pNavMeshPolygon->GetTrianglesFromLineIndex(lines->index).size() > 1)
		{
			NavGraphNode* nodeOnLine = {
				new NavGraphNode(GetNextFreeNodeIndex(),
				lines->index, Elite::Vector2{ (lines->p1 + lines->p2) / 2 }) };
			AddNode(nodeOnLine);
		}
	}

	//2. Create connections now that every node is created
	for (const auto& triangles : m_pNavMeshPolygon->GetTriangles())
	{
		std::vector<int> nodeIndexes{};

		for (const auto& lineIdx : triangles->metaData.IndexLines)
		{
			for(const auto node : m_Nodes)
			{
				if (node->GetLineIndex() == lineIdx) 
				{
					nodeIndexes.push_back(node->GetIndex());
				}
			}
		}

		if (nodeIndexes.size() == 2)
		{
			AddConnection(new GraphConnection2D(nodeIndexes[0], nodeIndexes[1]));
		}
		else if (nodeIndexes.size() == 3)
		{
			AddConnection(new GraphConnection2D(nodeIndexes[0], nodeIndexes[1]));
			AddConnection(new GraphConnection2D(nodeIndexes[1], nodeIndexes[2]));
			AddConnection(new GraphConnection2D(nodeIndexes[2], nodeIndexes[0]));
		}
	}

	//3. Set the connections cost to the actual distance
	SetConnectionCostsToDistance();
}

