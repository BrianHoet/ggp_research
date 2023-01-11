#include "stdafx.h"
#include "SpacePartitioning.h"
#include "projects\Movement\SteeringBehaviors\SteeringAgent.h"

// --- Cell ---
// ------------
Cell::Cell(float left, float bottom, float width, float height)
{
	boundingBox.bottomLeft = { left, bottom };
	boundingBox.width = width;
	boundingBox.height = height;
}

std::vector<Elite::Vector2> Cell::GetRectPoints() const
{
	auto left = boundingBox.bottomLeft.x;
	auto bottom = boundingBox.bottomLeft.y;
	auto width = boundingBox.width;
	auto height = boundingBox.height;

	std::vector<Elite::Vector2> rectPoints =
	{
		{ left , bottom  },
		{ left , bottom + height  },
		{ left + width , bottom + height },
		{ left + width , bottom  },
	};

	return rectPoints;
}

// --- Partitioned Space ---
// -------------------------
CellSpace::CellSpace(float width, float height, int rows, int cols, int maxEntities)
	: m_SpaceWidth(width)
	, m_SpaceHeight(height)
	, m_NrOfRows(rows)
	, m_NrOfCols(cols)
	, m_Neighbors(maxEntities)
	, m_NrOfNeighbors(0)
{
	//Get the dimensions of a cell
	m_CellWidth =  m_SpaceWidth / m_NrOfCols;
	m_CellHeight = m_SpaceHeight / m_NrOfRows;

	//Initialize the cells
	for(int i = 0; i <= m_NrOfRows; ++i)
	{
		for(int j = 0; j <= m_NrOfCols; ++j)
		{
			Elite::Vector2 pos{m_CellWidth * j,m_CellHeight * i};

			m_Cells.push_back(Cell{ pos.x, pos.y, m_CellWidth, m_CellHeight });
		}
	}
}

void CellSpace::AddAgent(SteeringAgent* agent)
{
	//		Gets the index of cell that agent's in		 Adds agent to list
	m_Cells[PositionToIndex(agent->GetPosition())].agents.push_back(agent);
}

void CellSpace::UpdateAgentCell(SteeringAgent* agent, Elite::Vector2 oldPos)
{
	const int newCell{ PositionToIndex(agent->GetPosition()) };
	const int oldCell{ PositionToIndex(oldPos) };

	if(newCell != oldCell)
	{
		m_Cells[oldCell].agents.remove(agent);
		m_Cells[newCell].agents.push_back(agent);
	}
}

void CellSpace::RegisterNeighbors(SteeringAgent* agent, float queryRadius, std::vector<SteeringAgent*>* neighbors)
{
	neighbors->clear();

	//Clamp to prevent from going out of bounds
	const int colB{ Elite::Clamp(int(agent->GetPosition().x - queryRadius), 0, m_NrOfRows - 1)};
	const int colE{ Elite::Clamp(int(agent->GetPosition().x + queryRadius), 0, m_NrOfRows - 1) };
	const int rowB{ Elite::Clamp(int(agent->GetPosition().y - queryRadius), 0, m_NrOfCols - 1) };
	const int rowE{ Elite::Clamp(int(agent->GetPosition().y + queryRadius), 0, m_NrOfCols - 1) };

	for (int  jdxRow = rowB; jdxRow <= colE; ++jdxRow)
	{
		for (int idxCol = colB; idxCol <= rowE; ++idxCol)
		{
			const int idxCell = idxCol + jdxRow * m_NrOfRows;

			for(SteeringAgent* pAgent : m_Cells[idxCell].agents)
			{
				if (pAgent != agent)
				{
					Elite::Vector2 distance{ pAgent->GetPosition() - agent->GetPosition() };
					if (distance.Magnitude() <= queryRadius)
					{
						neighbors->push_back(pAgent);
					}
				}
			}

		}
	}
}

void CellSpace::EmptyCells()
{
	for (Cell& c : m_Cells)
		c.agents.clear();
}

void CellSpace::RenderCells() const
{
	for (int i{}; i < m_Cells.size(); ++i)
	{
		std::vector<Elite::Vector2> points = m_Cells[i].GetRectPoints();
		auto polygon = Elite::Polygon{ points };
		DEBUGRENDERER2D->DrawPolygon(&polygon, { 1,0,0 }, 1.f);

		const std::string text{ std::to_string(m_Cells[i].agents.size()) };
		DEBUGRENDERER2D->DrawString(
			{m_Cells[i].boundingBox.bottomLeft.x,
			m_Cells[i].boundingBox.bottomLeft.y + m_Cells[i].boundingBox.height },
			text.c_str());
	}

}

int CellSpace::PositionToIndex(const Elite::Vector2 pos) const
{
	int idxColumn = pos.x / m_CellWidth;
	int idxRow = pos.y / m_CellHeight;

	//Makes sure if due to fps,
	//that the index stays in boundaries
	if (idxColumn >= m_NrOfCols)
		idxColumn = m_NrOfCols - 1;

	if (idxColumn < 0)
		idxColumn = 0;

	if (idxRow >= m_NrOfRows)
		idxRow = m_NrOfRows - 1;

	if (idxRow < 0)
		idxRow = 0;

	return 	(idxRow * m_NrOfCols + idxColumn);
}