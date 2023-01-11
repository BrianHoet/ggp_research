#include "stdafx.h"
#include "FlockingSteeringBehaviors.h"
#include "Flock.h"
#include "../SteeringAgent.h"
#include "../SteeringHelpers.h"


//*******************
//COHESION (FLOCKING)
SteeringOutput Cohesion::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering{};
	const Elite::Vector2 direction = m_pFlock->GetAverageNeighborPos() - pAgent->GetPosition();

	steering.LinearVelocity = direction;
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();
	return steering;
}

//*********************
//SEPARATION (FLOCKING)
SteeringOutput Separation::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering{};
	Elite::Vector2 direction{};
	Elite::Vector2 endDirection{};

	for(int i = 0; i < m_pFlock->GetNrOfNeighbors(); ++i)
	{
		direction = m_pFlock->GetNeighbors()[i]->GetPosition() - pAgent->GetPosition();
		endDirection -= direction;
	}

	steering.LinearVelocity = endDirection;
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();
	return steering;
}


//*************************
//VELOCITY MATCH (FLOCKING)
SteeringOutput VelocityMatch::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering{};

	steering.LinearVelocity = m_pFlock->GetAverageNeighborVelocity();
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();
	return steering;
}

//*************************
//EVADE (FLOCKING)
SteeringOutput EvadeFlocking::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering = {};
	const SteeringAgent* pAgentToEvade{ m_pFlock->GetAgentToEvade() };
	const Elite::Vector2 evadeMagnitude{ (pAgent->GetPosition() - pAgentToEvade->GetPosition()) };

	if (evadeMagnitude.Magnitude() < 10.f)
	{

		steering.LinearVelocity = evadeMagnitude;
		steering.LinearVelocity.Normalize();
		steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();
		steering.IsValid = true;
	}
	else
	{
		steering.IsValid = false;
	}

	return steering;
}
