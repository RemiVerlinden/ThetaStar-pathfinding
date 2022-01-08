#pragma once

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

		//https://www.geeksforgeeks.org/a-search-algorithm/

		vector<T_NodeType*> path;
		std::vector<NodeRecord> openList; // Node we still have to check
		std::vector<NodeRecord> closedList; // already checked nodes and history
		NodeRecord currentRecord{ pStartNode };
		currentRecord.estimatedTotalCost = GetHeuristicCost(pStartNode, pGoalNode);
		openList.push_back({ currentRecord });
		// Creating first record
		// Staring first record

		while (!openList.empty())
		{

			std::vector<NodeRecord>::iterator it_CurrentRecord{};
			size_t counter{};
			for (const NodeRecord& data : openList)
			{
				++counter;
				if (data < currentRecord) 
				{
					currentRecord = data;
					it_CurrentRecord = openList.begin() + counter;
				}
			}
			openList.erase(it_CurrentRecord);
			for (auto con : m_pGraph->GetNodeConnections(currentRecord.pNode))
			{
				NodeRecord neighbor{ m_pGraph->GetNode(con->GetTo()), con };
				if (neighbor.pNode == pGoalNode)
				{
					closedList.push_back(neighbor);
					break;
				}
				neighbor.costSoFar = currentRecord.costSoFar + con->GetCost();
				neighbor.estimatedTotalCost = neighbor.costSoFar + GetHeuristicCost(neighbor.pNode, pGoalNode);

				bool temp{};
					for (size_t i = 0; i < openList.size(); ++i)
					{
						if (openList[i].pNode == neighbor.pNode && openList[i] < neighbor)
							temp = true;
					}

					for (size_t i = 0; i < closedList.size(); ++i)
					{
						if (closedList[i].pNode == neighbor.pNode && closedList[i] < neighbor)
							temp = true;
					}

					if (temp)
					{
						continue;
					}

				openList.push_back(neighbor);
			}
			closedList.push_back(currentRecord);
		}

		// Tracking back when found destination
		for (size_t i = 0 ; i < closedList.size(); ++i)
		{
			path.push_back(closedList[i].pNode);
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