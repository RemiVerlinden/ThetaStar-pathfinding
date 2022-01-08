/*=============================================================================*/
// Copyright 2021-2022 Elite Engine
// Authors: Matthieu Delaere, Thomas Goussaert
/*=============================================================================*/
// SteeringBehaviors.h: SteeringBehaviors interface and different implementations
/*=============================================================================*/
#ifndef ELITE_STEERINGBEHAVIORS
#define ELITE_STEERINGBEHAVIORS

//-----------------------------------------------------------------
// Includes & Forward Declarations
//-----------------------------------------------------------------
#include "../SteeringHelpers.h"
class SteeringAgent;
class Obstacle;

#pragma region **ISTEERINGBEHAVIOR** (BASE)
class ISteeringBehavior
{
public:
	ISteeringBehavior() = default;
	virtual ~ISteeringBehavior() = default;

	virtual SteeringOutput CalculateSteering(float deltaT, SteeringAgent* pAgent) = 0;

	//Seek Functions
	void SetTarget(const TargetData& target) { m_Target = target; }

	template<class T, typename std::enable_if<std::is_base_of<ISteeringBehavior, T>::value>::type* = nullptr>
	T* As()
	{
		return static_cast<T*>(this);
	}

protected:
	TargetData m_Target;
};
#pragma endregion

///////////////////////////////////////
//SEEK
//****
class Seek : public ISteeringBehavior
{
public:
	Seek() = default;
	virtual ~Seek() = default;

	//Seek Behaviour
	SteeringOutput CalculateSteering(float deltaT, SteeringAgent* pAgent) override;
};

/////////////////////////
//FLEE
//****
class Flee : public ISteeringBehavior
{
public:
	Flee() = default;
	virtual ~Flee() = default;

	//Seek Behavior
	SteeringOutput CalculateSteering(float deltaT, SteeringAgent* pAgent) override;
private:
	float m_FleeRadius = 10.f;
};

/////////////////////////
//ARRIVE
//****
class Arrive : public ISteeringBehavior
{
public:
	Arrive() = default;
	virtual ~Arrive() = default;

	//Seek behavior
	SteeringOutput CalculateSteering(float deltaT, SteeringAgent* pAgent) override;
private:
	const float m_ArrivalRadius = 1.f;
	const float m_SlowRadius = 15.f;
	float distance = 0.f;
};

/////////////////////////
//FACE
//****
class Face : public ISteeringBehavior
{
public:
	Face() = default;
	virtual ~Face() = default;

	//Seek behavior
	SteeringOutput CalculateSteering(float deltaT, SteeringAgent* pAgent) override;
private:
	float Lerp(float a, float b, float f);
	float LerpRadians(float a, float b, float lerpPercent); // Lerps from angle a to b (both between 0.f and M_PI_TIMES_TWO), taking the shortest path

	float m_FaceSpeed{ 1.f };
};

/////////////////////////
//WANDER
//****
class Wander : public Seek
{
public:
	Wander() = default;
	virtual ~Wander() = default;

	//Wander behavior
	SteeringOutput CalculateSteering(float deltaT, SteeringAgent* pAgent) override;

	void SetWanderOffset(float offset) { m_OffsetDistance = offset; }
	void SetWanderRadius(float radius) { m_Radius = radius; }
	void SetMaxAnleChange(float rad) { m_MaxAngleChange = rad; }

private:
	float m_OffsetDistance{ 6.f }; // Offset (Agent direction)
	float m_Radius{ 4.f }; // Wander radius
	float m_MaxAngleChange = Elite::ToRadians(45); // Max WanderAngle change per frame
	float m_WanderAngle{ (float)M_PI / 2 }; // Internal
};

/////////////////////////
//Pursuit
//****
class Pursuit : public ISteeringBehavior
{
public:
	Pursuit() = default;
	virtual ~Pursuit() = default;

	virtual SteeringOutput CalculateSteering(float deltaT, SteeringAgent* pAgent);

protected:
	virtual Elite::Vector2 CalcPursuitTarget(SteeringAgent* pAgent);
	bool QuadraticSolver(float a, float b, float c, float& solution1, float& solution2);

};


/////////////////////////
//Evade
//****
class Evade :public Pursuit
{
public:
	Evade() = default;
	~Evade() = default;

	SteeringOutput CalculateSteering(float deltaT, SteeringAgent* pAgent) override;
private:
	Elite::Vector2 CalcPursuitTarget(SteeringAgent* pAgent) override;

	float m_EvadeRadius = 20.f;
};
#endif


