//Precompiled Header [ALWAYS ON TOP IN CPP]
#include "stdafx.h"

//Includes
#include "SteeringBehaviors.h"
#include "../SteeringAgent.h"
#include "../Obstacle.h"
#include "framework\EliteMath\EMatrix2x3.h"
#include <algorithm>
//SEEK
//****
SteeringOutput Seek::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering = {};

	steering.LinearVelocity = m_Target.Position - pAgent->GetPosition();
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();


	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, 5.f, Elite::Color{ 0, 1, 0, 1 });

		// calculate The current velocity of the agent and orientation
		float speed = pAgent->GetLinearVelocity().Magnitude() * 8.f / pAgent->GetMaxLinearSpeed();
			float agentRotation{ pAgent->GetRotation() - float(M_PI / 2) }; // rotation in rad
		Elite::Vector2 LookAtVector{ std::cos(agentRotation) // rotation visualised in vector form
								,std::sin(agentRotation) };
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), LookAtVector
			, speed, Elite::Color{ 1, 0, 1, 1 });
		Elite::Vector2 desiredVelocity{ steering.LinearVelocity - pAgent->GetLinearVelocity() };
		float magnitude = desiredVelocity.Magnitude();
		magnitude = (magnitude > 5.f) ? 5.f : magnitude;
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), desiredVelocity
			, magnitude, Elite::Color{ 0.4f, 0.9f, 1.f, 1 });
	}

	return steering;
}

//FLEE
//****
SteeringOutput Flee::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	float distanceToTagert = Distance(pAgent->GetPosition(), m_Target.Position);
	if (distanceToTagert > m_FleeRadius)
	{
		return SteeringOutput(Elite::ZeroVector2, 0.f, false);
	}
	SteeringOutput steering = { };

	steering.LinearVelocity = m_Target.Position - pAgent->GetPosition();
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= -pAgent->GetMaxLinearSpeed();

	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, 5.f, Elite::Color{ 0, 1, 0, 1 });

		// calculate The current velocity of the agent and orientation
		float speed = pAgent->GetLinearVelocity().Magnitude();
		float agentRotation{ pAgent->GetRotation() - float(M_PI / 2) }; // rotation in rad
		Elite::Vector2 LookAtVector{ std::cos(agentRotation) // rotation visualised in vector form
								,std::sin(agentRotation) };
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), LookAtVector
			, speed, Elite::Color{ 1, 0, 1, 1 });
		Elite::Vector2 desiredVelocity{ steering.LinearVelocity - pAgent->GetLinearVelocity() };
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), desiredVelocity
			, desiredVelocity.Magnitude(), Elite::Color{ 0.4f, 0.9f, 1.f, 1 });
	}

	return steering;
}


SteeringOutput Arrive::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering{};

	distance = Elite::Vector2(m_Target.Position - pAgent->GetPosition()).Magnitude();
	if (distance < m_ArrivalRadius)
	{
		steering.LinearVelocity = Elite::ZeroVector2;
	}
	else
	{
		steering.LinearVelocity = m_Target.Position - pAgent->GetPosition();
		steering.LinearVelocity.Normalize();

		if (distance < m_SlowRadius)
		{
			steering.LinearVelocity *= pAgent->GetMaxLinearSpeed() * distance / m_SlowRadius;
		}
		else
		{
			steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();
		}
	}

	float debugLineLength = steering.LinearVelocity.Magnitude() / pAgent->GetMaxLinearSpeed();
	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, 5.f * debugLineLength, Elite::Color{ 0, 1, 0, 1 });
		DEBUGRENDERER2D->DrawCircle(pAgent->GetPosition(), m_SlowRadius, Elite::Color(0.1f, 1.f, 1.f), 0.f); // slow radius
		DEBUGRENDERER2D->DrawCircle(pAgent->GetPosition(), m_ArrivalRadius, Elite::Color(1.f, 0.f, 0.f), 0.f); // stop radius

				// calculate The current velocity of the agent and orientation
		float speed = pAgent->GetLinearVelocity().Magnitude();
		float agentRotation{ pAgent->GetRotation() - float(M_PI / 2) }; // rotation in rad
		Elite::Vector2 LookAtVector{ std::cos(agentRotation) // rotation visualised in vector form
								,std::sin(agentRotation) };
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), LookAtVector
			, speed, Elite::Color{ 1, 0, 1, 1 });
		Elite::Vector2 desiredVelocity{ steering.LinearVelocity - pAgent->GetLinearVelocity() };
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), desiredVelocity
			, desiredVelocity.Magnitude(), Elite::Color{ 0.4f, 0.9f, 1.f, 1 });
	}

	return steering;
}

SteeringOutput Face::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering{};

	pAgent->SetAutoOrient(false);

	Elite::Vector2 direction{ m_Target.Position - pAgent->GetPosition() };
	float targetDirection{ Elite::GetOrientationFromVelocity(direction) };
	float orientation{ pAgent->GetOrientation() };

	// the first value in my the lerp is always 0 because
	// I want to get the angle of the target relative to the agent
	if ((targetDirection - orientation) < (float)M_PI || (targetDirection - orientation) > (float)M_PI)
	{
		steering.AngularVelocity = LerpRadians(0.f, targetDirection - orientation, deltaT) * 500.f * m_FaceSpeed;
	}
	else
	{
		steering.AngularVelocity = LerpRadians(0.f, orientation - targetDirection, deltaT) * 500.f * m_FaceSpeed;
	}


	if (pAgent->CanRenderBehavior())
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, 5.f, Elite::Color{ 0, 1, 0, 1 });

	return steering;
}

float Face::Lerp(float a, float b, float f)
{
	return (a * (1.0f - f)) + (b * f);
}

#define M_PI_TIMES_TWO M_PI*2.f

float Face::LerpRadians(float a, float b, float lerpPercent)
{

	float result{};
	float diff = b - a;

	if (diff < -M_PI)
	{
		b += (float)M_PI_TIMES_TWO;
		result = Lerp(a, b, lerpPercent);
	}
	else if (diff > M_PI)
	{
		b -= (float)M_PI_TIMES_TWO;
		result = Lerp(a, b, lerpPercent);
	}
	else
	{
		result = Lerp(a, b, lerpPercent);
	}
	return result;
}

SteeringOutput Wander::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{

	m_WanderAngle += (Elite::randomFloat(m_MaxAngleChange) - m_MaxAngleChange / 2);
	Elite::Vector2 targetRelativeToAgent{ std::cos(m_WanderAngle) * m_Radius, std::sin(m_WanderAngle) * m_Radius };
	float agentRotation{ pAgent->GetRotation() - float(M_PI / 2) };

	Elite::Vector2 circleCenter{ pAgent->GetPosition().x + std::cos(agentRotation) * m_OffsetDistance
								,pAgent->GetPosition().y + std::sin(agentRotation) * m_OffsetDistance };

	m_Target = Elite::Vector2{ circleCenter.x + targetRelativeToAgent.x
							  ,circleCenter.y + targetRelativeToAgent.y };


	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawCircle(circleCenter, m_Radius, Elite::Color(0.1f, 0.1f, 1.f), 0.f); // circle in front of agent
		DEBUGRENDERER2D->DrawCircle(m_Target.Position, 0.2f, Elite::Color(0, 1, 0), 0.f); // target location
		//DEBUGRENDERER2D->DrawSegment(pAgent->GetPosition(), circleCenter, Elite::Color(0.7f, 0.3f, 1.f)); // line between agent and center of circle
	}

	SteeringOutput steering{ Seek::CalculateSteering(deltaT, pAgent) };


	return steering;
}

SteeringOutput Pursuit::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering{};
	Elite::Vector2 interceptPos = CalcPursuitTarget(pAgent);

	steering.LinearVelocity = interceptPos - pAgent->GetPosition();
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();
	
	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawSolidCircle(interceptPos, 2.f, Elite::Vector2{0,0}, Elite::Color(0.9f, 0.9f, 0.9f)); // stop radius

				// calculate The current velocity of the agent and orientation
		float speed = pAgent->GetLinearVelocity().Magnitude();
		float agentRotation{ pAgent->GetRotation() - float(M_PI / 2) }; // rotation in rad
		Elite::Vector2 LookAtVector{ std::cos(agentRotation) // rotation visualised in vector form
								,std::sin(agentRotation) };
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), LookAtVector
			, speed, Elite::Color{ 1, 0, 1, 1 });
		DEBUGRENDERER2D->DrawSegment(m_Target.Position, interceptPos, Elite::Color{ 0.9f, 0.9f, 0.9f, 0.3f });
		Elite::Vector2 desiredVelocity{ steering.LinearVelocity - pAgent->GetLinearVelocity() };
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), desiredVelocity
			, desiredVelocity.Magnitude(), Elite::Color{ 0.4f, 0.9f, 1.f, 1 });
	}
	return steering;
}
//------------------------------------------------------------------------------------------------------
//https://www.codeproject.com/Articles/990452/Interception-of-Two-Moving-Objects-in-D-Space#_articleTop
//
bool Pursuit::QuadraticSolver(float a, float b, float c, float& solution1, float& solution2)
{
	if (a == 0)
	{
		if (b == 0)
		{
			solution1 = solution2 = 0;
			return false;
		}
		else
		{
			solution1 = solution2 = -c / b;
			return true;
		}
	}

	float discrimintant = b * b - 4 * a * c;
	if (discrimintant < 0)
	{
		solution1 = solution2 = 0;
		return false;
	}

	discrimintant = std::sqrt(discrimintant);
	solution1 = (-b + discrimintant) / (2 * a);
	solution2 = (-b - discrimintant) / (2 * a);
	return true;
}

Elite::Vector2 Pursuit::CalcPursuitTarget(SteeringAgent* pAgent)
{
	float timeToInterception = 0.f;
	Elite::Vector2 interceptPos{};

	Elite::Vector2 vectorFromRunner = pAgent->GetPosition() - m_Target.Position;
	float distanceToRunner = vectorFromRunner.Magnitude();
	float runnerSpeed = m_Target.LinearVelocity.Magnitude();

	//else // Everything looks OK for the Law of Cosines approach
	//{
		// Now set up the quadratic formula coefficients
	float a = pAgent->GetMaxLinearSpeed() * pAgent->GetMaxLinearSpeed() - runnerSpeed * runnerSpeed;
	float b = 2 * vectorFromRunner.Dot(m_Target.LinearVelocity);
	float c = -distanceToRunner * distanceToRunner;

	float t1{}, t2{};
	if (!QuadraticSolver(a, b, c, t1, t2))
	{
		interceptPos = m_Target.LinearVelocity * timeToInterception  + m_Target.Position;
		return interceptPos; //when discriminant < 0 then abort
	}
	if (t1 < 0 && t2 < 0)
	{
		interceptPos = m_Target.LinearVelocity * timeToInterception  + m_Target.Position;
		return interceptPos;
	}
	if (t1 > 0 && t2 > 0) // We need the smallest one take the smaller value
		timeToInterception = (std::min)(t1, t2);
	else // One has to be negative, so take the larger one
		timeToInterception = (std::max)(t1, t2);

	interceptPos = m_Target.LinearVelocity * timeToInterception  + m_Target.Position;
	return interceptPos;
}
//------------------------------------------------------------------------------------------------------

SteeringOutput Evade::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{

	float distanceToTagert = Distance(pAgent->GetPosition(), m_Target.Position);
	if (distanceToTagert > m_EvadeRadius )
	{
		return SteeringOutput(Elite::ZeroVector2, 0.f, false);
	}

	SteeringOutput steering{};
	Elite::Vector2 interceptPos = CalcPursuitTarget(pAgent);

	steering.LinearVelocity = -(interceptPos - pAgent->GetPosition());
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();

	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawSolidCircle(interceptPos, 2.f, Elite::Vector2{ 0,0 }, Elite::Color(0.9f, 0.9f, 0.9f)); // stop radius

				// calculate The current velocity of the agent and orientation
		float speed = pAgent->GetLinearVelocity().Magnitude();
		float agentRotation{ pAgent->GetRotation() - float(M_PI / 2) }; // rotation in rad
		Elite::Vector2 LookAtVector{ std::cos(agentRotation) // rotation visualised in vector form
								,std::sin(agentRotation) };
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), LookAtVector
			, speed, Elite::Color{ 1, 0, 1, 1 });
		DEBUGRENDERER2D->DrawSegment(m_Target.Position, interceptPos, Elite::Color{ 0.9f, 0.9f, 0.9f, 0.3f });
	}
	return steering;
}

Elite::Vector2 Evade::CalcPursuitTarget(SteeringAgent* pAgent)
{
	return Pursuit::CalcPursuitTarget(pAgent);
}

