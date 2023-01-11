#pragma once
#include <vector>
#include "framework/EliteAI/EliteNavigation/ENavigation.h"

namespace Elite
{
	template <class T_NodeType, class T_ConnectionType>
	class AStar
	{
	public:
		AStar(IGraph<T_NodeType, T_ConnectionType>* pGraph, Heuristic hFunction);

		// stores the optimal connection to a node and its total costs related to the start and end node of the path
		struct NodeRecord
		{
			T_NodeType* pNode = nullptr;
			T_ConnectionType* pConnection = nullptr;
			float costSoFar = 0.f; // accumulated g-costs of all the connections leading up to this one
			float estimatedTotalCost = 0.f; // f-cost (= costSoFar + h-cost)

			bool operator==(const NodeRecord& other) const
			{
				return pNode == other.pNode
					&& pConnection == other.pConnection
					&& costSoFar == other.costSoFar
					&& estimatedTotalCost == other.estimatedTotalCost;
			};

			bool operator<(const NodeRecord& other) const
			{
				return estimatedTotalCost < other.estimatedTotalCost;
			};
		};

		std::vector<T_NodeType*> FindPath(T_NodeType* pStartNode, T_NodeType* pDestinationNode);

	private:
		float GetHeuristicCost(T_NodeType* pStartNode, T_NodeType* pEndNode) const;

		IGraph<T_NodeType, T_ConnectionType>* m_pGraph;
		Heuristic m_HeuristicFunction;
	};

	template <class T_NodeType, class T_ConnectionType>
	AStar<T_NodeType, T_ConnectionType>::AStar(IGraph<T_NodeType, T_ConnectionType>* pGraph, Heuristic hFunction)
		: m_pGraph(pGraph)
		, m_HeuristicFunction(hFunction)
	{
	}

	template <class T_NodeType, class T_ConnectionType>
	std::vector<T_NodeType*> AStar<T_NodeType, T_ConnectionType>::FindPath(T_NodeType* pStartNode, T_NodeType* pGoalNode)
	{
		std::vector<T_NodeType*> path;
		std::vector<NodeRecord> openList;
		std::vector<NodeRecord> closedList;
		NodeRecord currentRecord;
		
		openList.push_back(NodeRecord{pStartNode, nullptr, 0.f, GetHeuristicCost(pStartNode, pGoalNode)});

		while (!openList.empty())
		{
			currentRecord = *std::min_element(openList.begin(), openList.end());
			if (currentRecord.pNode == pGoalNode)
			{
				break;
			}
			for (T_ConnectionType* const pCon : m_pGraph->GetNodeConnections(currentRecord.pNode))
			{
				T_NodeType* pNode{ m_pGraph->GetNode(pCon->GetTo()) };
				auto IsNodeInList = NodeRecord{ pNode, pCon, currentRecord.costSoFar + pCon->GetCost(), currentRecord.costSoFar + pCon->GetCost() + GetHeuristicCost(pNode, pGoalNode) };
				auto checkFind = std::find_if(closedList.begin(), closedList.end(), [&](NodeRecord Node) {return Node.pNode == pNode; });

				if (checkFind != closedList.end())
				{
					if (checkFind->costSoFar < IsNodeInList.costSoFar)
					{
						continue;
					}
				}
				else
				{
					checkFind = std::find_if(openList.begin(), openList.end(), [&](NodeRecord Node) {return Node.pNode == pNode; });
					if (checkFind != openList.end())
					{
						if (checkFind->costSoFar < IsNodeInList.costSoFar)
						{
							continue;
						}
						else
						{
							openList.erase(checkFind);
						}
					}
				}
				openList.push_back(IsNodeInList);
			}
			openList.erase(std::remove(openList.begin(), openList.end(), currentRecord));
			closedList.push_back(currentRecord);
		}

		//We reached the goal node
		//Start backtracking
		if (currentRecord.pNode == pGoalNode)
		{

			while (currentRecord.pNode != pStartNode)
			{
				path.push_back(currentRecord.pNode);
				T_NodeType* pStartNode = m_pGraph->GetNode(currentRecord.pConnection->GetFrom());
				auto x = std::find_if(closedList.begin(), closedList.end(), [&](NodeRecord Node) {return Node.pNode == pStartNode; });
				
				if (x != closedList.end())
				{
					currentRecord = *x;
				}
				
			}

			path.push_back(pStartNode);
			std::reverse(path.begin(), path.end());
		}
		return path;
	}

	template <class T_NodeType, class T_ConnectionType>
	float Elite::AStar<T_NodeType, T_ConnectionType>::GetHeuristicCost(T_NodeType* pStartNode, T_NodeType* pEndNode) const
	{
		Vector2 toDestination = m_pGraph->GetNodePos(pEndNode) - m_pGraph->GetNodePos(pStartNode);
		return m_HeuristicFunction(abs(toDestination.x), abs(toDestination.y));
	}
}