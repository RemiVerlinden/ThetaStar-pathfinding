#pragma once
#include "../SteeringHelpers.h"
#include "FlockingSteeringBehaviors.h"
#include "..\SpacePartitioning\SpacePartitioning.h"

class ISteeringBehavior;
class SteeringAgent;
class BlendedSteering;
class PrioritySteering;


//#define USE_SPACE_PARTITIONING

class Flock
{
public:
	Flock(
		int flockSize = 50, 
		float worldSize = 100.f,
		SteeringAgent* pAgentToEvade = nullptr, 
		bool trimWorld = false);

	~Flock();

	void Update(float deltaT);
	void UpdateAndRenderUI() ;
	void Render(float deltaT);

	void RegisterNeighbors(SteeringAgent* pAgent);
	int GetNrOfNeighbors() const { return m_NrOfNeighbors; }
	const vector<SteeringAgent*>& GetNeighbors() const { return m_Neighbors; }

	Elite::Vector2 GetAverageNeighborPos() const;
	Elite::Vector2 GetAverageNeighborVelocity() const;

	void SetSeekTarget(TargetData target);
	void SetWorldTrimSize(float size) { m_WorldSize = size; }
	float GetWorldTrimSize() { return m_WorldSize; }
	bool GetTrimWorld() const { return m_TrimWorld; }
	float GetNeighborhoodRadius() { return m_NeighborhoodRadius;  }
private:
	//Datamembers
	int m_FlockSize = 0;
	vector<SteeringAgent*> m_Agents;
	vector<Elite::Vector2> m_oldAgentPos;
	vector<SteeringAgent*> m_Neighbors;
	CellSpace* m_CellSpace;

	bool m_DebugRenderSteering = false;
	bool m_DebugRenderNeighbor = true;
	bool m_TrimWorld = false;
	bool m_DisableRedAgent = true;
	float m_WorldSize = 100.f;

	bool m_SpatialPartitioning = false;

	float m_NeighborhoodRadius = 15.f;
	int m_NrOfNeighbors = 0;

	SteeringAgent* m_pAgentToEvade = nullptr;
	
	//Steering Behaviors
	Separation* m_pSeparationBehavior = nullptr;
	Cohesion* m_pCohesionBehavior = nullptr;
	VelocityMatch* m_pVelMatchBehavior = nullptr;
	Seek* m_pSeekBehavior = nullptr;
	Wander* m_pWanderBehavior = nullptr;
	Evade* m_pEvadeBehavior = nullptr;

	BlendedSteering* m_pBlendedSteering = nullptr;
	PrioritySteering* m_pPrioritySteering = nullptr;

	float* GetWeight(ISteeringBehavior* pBehaviour);

	const Elite::Color m_Green = Elite::Color{ 0.f, 1.f, 0.f, 1.f };
	const Elite::Color m_Yellow = Elite::Color{ 1.f, 1.f, 0.f, 1.f };
	const Elite::Color m_White = Elite::Color{ 1.f, 1.f, 1.f, 1.f };


private:
	Flock(const Flock& other);
	Flock& operator=(const Flock& other);
};