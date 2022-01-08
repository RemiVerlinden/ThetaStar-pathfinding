#include "stdafx.h"
#include "FlockingSteeringBehaviors.h"
#include "TheFlock.h"
#include "../SteeringAgent.h"
#include "../SteeringHelpers.h"

//*******************
//COHESION (FLOCKING)
SteeringOutput Cohesion::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	m_Target.Position = m_pFlock->GetAverageNeighborPos();
	//DEBUGRENDERER2D->DrawCircle(pAgent->GetPosition(), 15.f, Elite::Color(1.f, 1.f, 1.f), 0.f); // slow radius
	return Seek::CalculateSteering(deltaT, pAgent);
}


//*********************
//SEPARATION (FLOCKING)
SteeringOutput Separation::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{ 
	Elite::Vector2 separation{};
	float neighboorhoodRadius{ m_pFlock->GetNeighborhoodRadius() };
	for (SteeringAgent* other : m_pFlock->GetNeighbors())
	{
		if (other == nullptr) break;
		separation += Elite::GetNormalized(other->GetPosition() - pAgent->GetPosition()) * -neighboorhoodRadius + other->GetPosition() - pAgent->GetPosition();
	}
	SteeringOutput steering = { };

	steering.LinearVelocity = separation;
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();

	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), separation
			, 7.f, Elite::Color{ 1.f, 0.4f, 0.1f, 1 });
	}
	return steering;
}

//*************************
//VELOCITY MATCH (FLOCKING)
SteeringOutput VelocityMatch::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{

	SteeringOutput steering = { };
	Elite::Vector2 avgVelocity = m_pFlock->GetAverageNeighborVelocity();

	steering.LinearVelocity = avgVelocity;
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();
	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), avgVelocity
			, 7.f, Elite::Color{ 1.f, 1.f, 1.f, 1 });
	}
	return steering;
}