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
	m_CellWidth = m_SpaceWidth / m_NrOfRows;
	m_CellHeight = m_SpaceHeight / m_NrOfCols;

	Elite::Vector2 botLeftPos = { -width / 2, -height/2 };
	for (size_t column = 0; column < m_NrOfCols; ++column)
	{
		for (size_t row = 0; row < m_NrOfRows; ++row)
		{
			m_Cells.push_back({ botLeftPos.x + row * m_CellWidth, botLeftPos.y + column * m_CellHeight, m_CellWidth, m_CellHeight });
		}
	}
}

void CellSpace::AddAgent(SteeringAgent* agent)
{
	m_Cells[PositionToIndex(agent->GetPosition())].agents.push_back(agent);
}

void CellSpace::UpdateAgentCell(SteeringAgent* agent, Elite::Vector2 oldPos)
{
	int oldIndex{ PositionToIndex(oldPos) }, newIndex{ PositionToIndex(agent->GetPosition()) };
	if (oldIndex != newIndex)
	{
		PositionToIndex(agent->GetPosition());

		m_Cells[oldIndex].agents.remove(agent);
		m_Cells[newIndex].agents.push_back(agent);
	}
}

void CellSpace::RegisterNeighbors(SteeringAgent* agent, float queryRadius)
{
	m_NrOfNeighbors = 0;
	std::fill(m_Neighbors.begin(), m_Neighbors.end(), nullptr);

	Elite::Rect agentBoundingBox{ agent->GetPosition() - Elite::Vector2{queryRadius, queryRadius}, queryRadius * 2, queryRadius * 2 };
	for ( const Cell& cell : m_Cells)
	{
		if (Elite::IsOverlapping(cell.boundingBox, agentBoundingBox ))
		{
			for (SteeringAgent* cellAgent : cell.agents)
			{
				float distanceToOther = Distance(agent->GetPosition(), cellAgent->GetPosition());
				if (distanceToOther < queryRadius)
				{
					m_Neighbors[m_NrOfNeighbors] = cellAgent;
					++m_NrOfNeighbors;
				}
			}
		}
	}
}

void CellSpace::RenderCells() const
{
	for (const Cell& index : m_Cells)
	{
		DEBUGRENDERER2D->DrawPolygon(&index.GetRectPoints()[0], 4, { 1,0,0,1 }, 0.4f);
	}
}

int CellSpace::PositionToIndex(const Elite::Vector2 pos) const
{
	int row = (m_SpaceWidth / 2 + pos.x) / m_CellWidth;
	int col = (m_SpaceHeight / 2 + pos.y) / m_CellHeight;
	if (row >= m_NrOfRows) row = 0;
	if (col >= m_NrOfCols) col = 0;
	if (row < 0) row = m_NrOfRows - 1;
	if (col < 0) col = m_NrOfCols - 1;
	return col * m_NrOfRows + row;
}