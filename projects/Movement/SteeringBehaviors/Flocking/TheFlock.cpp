#include "stdafx.h"
#include "TheFlock.h"

#include "../SteeringAgent.h"
#include "../Steering/SteeringBehaviors.h"
#include "../CombinedSteering/CombinedSteeringBehaviors.h"

//Constructor & Destructor
Flock::Flock(
	int flockSize /*= 50*/,
	float worldSize /*= 100.f*/,
	SteeringAgent* pAgentToEvade /*= nullptr*/,
	bool trimWorld /*= false*/)

	: m_WorldSize{ worldSize }
	, m_FlockSize{ flockSize }
	, m_TrimWorld{ trimWorld }
	, m_pAgentToEvade{ pAgentToEvade }
	, m_NeighborhoodRadius{ 5.f }
	, m_NrOfNeighbors{ 0 }
{
	m_FlockSize = 4000;
	m_CellSpace = new CellSpace{ worldSize * 2, worldSize * 2, 25, 25, m_FlockSize };
	m_Agents.resize(m_FlockSize);
	m_oldAgentPos.resize(m_FlockSize);

	// TODO: initialize the flock and the memory pool
	m_pSeparationBehavior = new Separation{ this };
	m_pCohesionBehavior = new Cohesion{ this };
	m_pVelMatchBehavior = new VelocityMatch{ this };
	m_pSeekBehavior = new Seek{};
	m_pWanderBehavior = new Wander{};
	m_pEvadeBehavior = new Evade{};
	vector < BlendedSteering::WeightedBehavior> weightedSteeringBehaviors;
	weightedSteeringBehaviors.push_back({ m_pSeparationBehavior, 0.2f });
	weightedSteeringBehaviors.push_back({ m_pCohesionBehavior, 0.2f });
	weightedSteeringBehaviors.push_back({ m_pVelMatchBehavior, 0.2f });
	weightedSteeringBehaviors.push_back({ m_pSeekBehavior, 0.2f });
	weightedSteeringBehaviors.push_back({ m_pWanderBehavior, 0.2f });
	m_pBlendedSteering = new BlendedSteering(weightedSteeringBehaviors);
	m_pPrioritySteering = new PrioritySteering({ m_pEvadeBehavior, m_pBlendedSteering });

	m_pAgentToEvade->SetBodyColor({ 1.f, 0.f, 0.f });
	m_pAgentToEvade->SetSteeringBehavior(m_pWanderBehavior);
	m_pAgentToEvade->SetMass(0.1f);
	m_pAgentToEvade->SetMaxAngularSpeed(25.f);
	m_pAgentToEvade->SetMaxLinearSpeed(35.f);
	m_pAgentToEvade->SetAutoOrient(true);

	for (SteeringAgent*& pAgent : m_Agents)
	{
		pAgent = new SteeringAgent{};
		pAgent->SetPosition({ rand() % ((int)worldSize * 2) - worldSize, rand() % ((int)worldSize * 2) - worldSize });
		pAgent->SetRotation(Elite::ToRadians(rand() % 180 + 180));
		pAgent->SetSteeringBehavior(m_pPrioritySteering);
		pAgent->SetMass(1.f);
		pAgent->SetMaxAngularSpeed(25.f);
		pAgent->SetMaxLinearSpeed(55.f);
		pAgent->SetAutoOrient(true);
		m_CellSpace->AddAgent(pAgent);
	}
	m_Neighbors.resize(m_FlockSize);
}

Flock::~Flock()
{
	// TODO: clean up any additional data
	SAFE_DELETE(m_CellSpace);
	SAFE_DELETE(m_pSeparationBehavior);
	SAFE_DELETE(m_pCohesionBehavior);
	SAFE_DELETE(m_pVelMatchBehavior);
	SAFE_DELETE(m_pSeekBehavior);
	SAFE_DELETE(m_pWanderBehavior);
	SAFE_DELETE(m_pEvadeBehavior);
	SAFE_DELETE(m_pPrioritySteering);

	SAFE_DELETE(m_pBlendedSteering);
	SAFE_DELETE(m_pPrioritySteering);

	for (auto pAgent : m_Agents)
	{
		SAFE_DELETE(pAgent);
	}
	m_Agents.clear();
}

void Flock::Update(float deltaT)
{
	// TODO: update the flock
	// loop over all the agents
		// register its neighbors	(-> memory pool is filled with neighbors of the currently evaluated agent)
		// update it				(-> the behaviors can use the neighbors stored in the pool, next iteration they will be the next agent's neighbors)
		// trim it to the world
	if (!m_DisableRedAgent)
	{
		m_pAgentToEvade->Update(deltaT);
		if (m_TrimWorld)
			m_pAgentToEvade->TrimToWorld(m_WorldSize);
	}
	else
	{
		m_pAgentToEvade->SetPosition({ m_WorldSize * 5, m_WorldSize * 5 });
	}

	m_pEvadeBehavior->SetTarget({ m_pAgentToEvade->GetPosition(), 0.f, m_pAgentToEvade->GetLinearVelocity(), m_pAgentToEvade->GetAngularVelocity() });
	int counter{};
	for (SteeringAgent* pAgent : m_Agents)
	{
		RegisterNeighbors(pAgent);
		m_CellSpace->UpdateAgentCell(pAgent, m_oldAgentPos[counter]);
		m_oldAgentPos[counter] = pAgent->GetPosition();
		pAgent->Update(deltaT);
		if (m_TrimWorld)
			pAgent->TrimToWorld(m_WorldSize);
		++counter;
	}
}

void Flock::Render(float deltaT)
{
	// TODO: render the flock
	m_pAgentToEvade->SetRenderBehavior(m_DebugRenderSteering);
	m_pAgentToEvade->Render(deltaT);
	m_CellSpace->RenderCells();

	for (SteeringAgent* pAgent : m_Agents)
	{
		if (m_DebugRenderNeighbor && pAgent == m_Agents[m_FlockSize - 1])
		{
			pAgent->SetRenderBehavior(true);
		}
		else
		{
			pAgent->SetRenderBehavior(m_DebugRenderSteering);
		}
		pAgent->Render(deltaT);
		pAgent->SetBodyColor(m_Yellow);
	}
	if (m_DebugRenderNeighbor)
	{
		DEBUGRENDERER2D->DrawCircle(m_Agents[m_FlockSize - 1]->GetPosition(), m_NeighborhoodRadius, m_White, 0.f);
		//if (m_SpatialPartitioning)
		//{
		//	for (SteeringAgent* pAgent : m_CellSpace->GetNeighbors())
		//	{
		//		if (pAgent == nullptr) return;
		//		pAgent->SetBodyColor(m_Green);
		//	}
		//}
		//else
		//{
			for (SteeringAgent* pAgent : m_Neighbors)
			{
				if (pAgent == nullptr) return;
				pAgent->SetBodyColor(m_Green);
			}
		//}
	}
}

void Flock::RegisterNeighbors(SteeringAgent* pAgent)
{
		std::fill(m_Neighbors.begin(), m_Neighbors.end(), nullptr);
		m_NrOfNeighbors = 0;

	if (m_SpatialPartitioning)
	{
		m_CellSpace->RegisterNeighbors(pAgent, m_NeighborhoodRadius);
		for (size_t i = 0; i < m_CellSpace->GetNrOfNeighbors(); ++i)
		{
			m_Neighbors[i] = m_CellSpace->GetNeighbors()[i];
		}
	}
	else
	{
		for (SteeringAgent* other : m_Agents)
		{
			float distanceToOther = Distance(pAgent->GetPosition(), other->GetPosition());
			if (distanceToOther < m_NeighborhoodRadius)
			{
				//m_Neighbors[m_Neighbors.size()] = other;
				m_Neighbors[m_NrOfNeighbors] = other;
				++m_NrOfNeighbors;
			}
		}
	}
}

void Flock::UpdateAndRenderUI()
{
	//Setup
	int menuWidth = 235;
	int const width = DEBUGRENDERER2D->GetActiveCamera()->GetWidth();
	int const height = DEBUGRENDERER2D->GetActiveCamera()->GetHeight();
	bool windowActive = true;
	ImGui::SetNextWindowPos(ImVec2((float)width - menuWidth - 10, 10));
	ImGui::SetNextWindowSize(ImVec2((float)menuWidth, (float)height - 20));
	ImGui::Begin("Gameplay Programming", &windowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
	ImGui::PushAllowKeyboardFocus(false);

	//Elements
	ImGui::Text("CONTROLS");
	ImGui::Indent();
	ImGui::Text("LMB: place target");
	ImGui::Text("RMB: move cam.");
	ImGui::Text("Scrollwheel: zoom cam.");
	ImGui::Unindent();

	ImGui::Spacing();
	ImGui::Separator();
	ImGui::Spacing();
	ImGui::Spacing();

	ImGui::Text("STATS");
	ImGui::Indent();
	ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
	ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
	ImGui::Unindent();

	ImGui::Spacing();
	ImGui::Separator();
	ImGui::Spacing();

	ImGui::Text("Flocking");
	ImGui::Spacing();
	ImGui::Spacing();

	// TODO: Implement checkboxes for debug rendering and weight sliders here
	ImGui::Checkbox("Debug Rendering Steering", &m_DebugRenderSteering);
	ImGui::Checkbox("Debug Rendering Neighborhood", &m_DebugRenderNeighbor);
	ImGui::Checkbox("Disable Red Agent", &m_DisableRedAgent);
	ImGui::Spacing();
	ImGui::Spacing();
	ImGui::Spacing();
	ImGui::Text("World Settings");
	ImGui::Spacing();
	ImGui::Checkbox("Spatial Partitioning", &m_SpatialPartitioning);
	ImGui::Checkbox("Trim World", &m_TrimWorld);
	if (m_TrimWorld)
	{
		ImGui::SliderFloat("Trim Size", &m_WorldSize, 0.f, 250.f, "%1.");
	}
	ImGui::Spacing();
	ImGui::Spacing();
	ImGui::Spacing();

	ImGui::Text("Behavior Weights");
	ImGui::Spacing();

	ImGui::SliderFloat("Separation", &m_pBlendedSteering->GetWeightedBehaviorsRef()[0].weight, 0.f, 1.f, "%.2");
	ImGui::SliderFloat("Cohesion", &m_pBlendedSteering->GetWeightedBehaviorsRef()[1].weight, 0.f, 1.f, "%.2");
	ImGui::SliderFloat("VelocityMatch", &m_pBlendedSteering->GetWeightedBehaviorsRef()[2].weight, 0.f, 1.f, "%.2");
	ImGui::SliderFloat("Seek", &m_pBlendedSteering->GetWeightedBehaviorsRef()[3].weight, 0.f, 1.f, "%.2");
	ImGui::SliderFloat("Wander", &m_pBlendedSteering->GetWeightedBehaviorsRef()[4].weight, 0.f, 1.f, "%.2");

	//End
	ImGui::PopAllowKeyboardFocus();
	ImGui::End();

}

Elite::Vector2 Flock::GetAverageNeighborPos() const
{
	Elite::Vector2 averagePos{};
	//if (m_SpatialPartitioning)
	//{
	//	for (SteeringAgent* pAgent : m_CellSpace->GetNeighbors())
	//	{
	//		if (pAgent == nullptr) break;
	//		averagePos += pAgent->GetPosition();
	//	}
	//}
	//else
	//{
		for (SteeringAgent* pAgent : m_Neighbors)
		{
			if (pAgent == nullptr) break;
			averagePos += pAgent->GetPosition();
		}
	//}
	averagePos /= m_NrOfNeighbors;
	return averagePos;
}

Elite::Vector2 Flock::GetAverageNeighborVelocity() const
{
	Elite::Vector2 averageVelocity{};
	if (m_NrOfNeighbors == 1) return Elite::ZeroVector2;
	//if (m_SpatialPartitioning)
	//{
	//	for (SteeringAgent* pAgent : m_CellSpace->GetNeighbors())
	//	{
	//		if (pAgent == nullptr) break;
	//		averageVelocity += pAgent->GetLinearVelocity();
	//	}
	//}
	//else
	//{
		for (SteeringAgent* pAgent : m_Neighbors)
		{
			if (pAgent == nullptr) break;
			averageVelocity += pAgent->GetLinearVelocity();
		}
	//}
	averageVelocity /= m_NrOfNeighbors;
	return averageVelocity;
}

void Flock::SetSeekTarget(TargetData target)
{
	// TODO: set target for Seek behavior
	m_pSeekBehavior->SetTarget(target);
}

float* Flock::GetWeight(ISteeringBehavior* pBehavior)
{
	if (m_pBlendedSteering)
	{
		auto& weightedBehaviors = m_pBlendedSteering->GetWeightedBehaviorsRef();
		auto it = find_if(weightedBehaviors.begin(),
			weightedBehaviors.end(),
			[pBehavior](BlendedSteering::WeightedBehavior el)
			{
				return el.pBehavior == pBehavior;
			}
		);

		if (it != weightedBehaviors.end())
			return &it->weight;
	}

	return nullptr;
}
