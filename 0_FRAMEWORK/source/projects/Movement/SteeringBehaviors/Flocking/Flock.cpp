#include "stdafx.h"
#include "Flock.h"

#include "../SteeringAgent.h"
#include "../Steering/SteeringBehaviors.h"
#include "../CombinedSteering/CombinedSteeringBehaviors.h"
#include "../SpacePartitioning/SpacePartitioning.h"

using namespace Elite;

//Constructor & Destructor
Flock::Flock(
	int flockSize /*= 50*/, 
	float worldSize /*= 100.f*/, 
	SteeringAgent* pAgentToEvade /*= nullptr*/, 
	bool trimWorld /*= false*/)

	: m_WorldSize{ worldSize }
	, m_FlockSize{ flockSize }
	, m_TrimWorld { trimWorld }
	, m_pAgentToEvade{pAgentToEvade}
	, m_NeighborhoodRadius{ 15 }
	, m_NrOfNeighbors{0}
{
	m_Agents.resize(m_FlockSize);

	// TODO: initialize the flock and the memory pool
	m_pCohesionBehavior = new Cohesion(this);
	m_pSeparationBehavior = new Separation(this);
	m_pVelMatchBehavior = new VelocityMatch(this);
	m_pWanderBehavior = new Wander();
	m_pEvadeBehavior = new EvadeFlocking(this);
	m_pSeekBehavior = new Seek();
	m_pCellSpace = new CellSpace(m_WorldSize, m_WorldSize, 30, 30, m_FlockSize);

	const std::vector<BlendedSteering::WeightedBehavior> blended{
		{m_pCohesionBehavior, 5},
		{m_pSeparationBehavior, 5},
		{m_pVelMatchBehavior, 5},
		{m_pSeekBehavior, 0},
		{m_pWanderBehavior, 0}
	};
	m_pBlendedSteering = new BlendedSteering(blended);

	//Order of behavior decide priority
	const std::vector<ISteeringBehavior*> prioritySteering{m_pEvadeBehavior, m_pBlendedSteering};
	m_pPrioritySteering = new PrioritySteering(prioritySteering);

	//Initialize all flock agents
	for(UINT i = 0; i < m_FlockSize; ++i)
	{
		m_Agents[i] = new SteeringAgent();
		m_Agents[i]->SetSteeringBehavior(m_pPrioritySteering);
		m_Agents[i]->SetAutoOrient(true);
		m_Agents[i]->SetMaxLinearSpeed(50.f);
		m_Agents[i]->SetMass(0.3f);
		m_Agents[i]->SetPosition({ float(rand() % int(worldSize + 1)), float(rand() % int(worldSize + 1)) });
		m_pCellSpace->AddAgent(m_Agents[i]);
	}

	//Initialize agent to evade
	m_pAgentToEvade = new SteeringAgent();
	m_pAgentToEvade->SetSteeringBehavior(m_pWanderBehavior);
	m_pAgentToEvade->SetAutoOrient(true);
	m_pAgentToEvade->SetMaxLinearSpeed(50.f);
	m_pAgentToEvade->SetMass(0.3f);
	m_pAgentToEvade->SetBodyColor({ 1,0,0 });
}

Flock::~Flock()
{
	// TODO: clean up any additional data
	SAFE_DELETE(m_pBlendedSteering);
	SAFE_DELETE(m_pPrioritySteering);
	SAFE_DELETE(m_pCohesionBehavior);
	SAFE_DELETE(m_pSeparationBehavior);
	SAFE_DELETE(m_pVelMatchBehavior);
	SAFE_DELETE(m_pWanderBehavior);
	SAFE_DELETE(m_pEvadeBehavior);
	SAFE_DELETE(m_pSeekBehavior);

	for(auto pAgent: m_Agents)
	{
		SAFE_DELETE(pAgent);
	}
	m_Agents.clear();

	SAFE_DELETE(m_pAgentToEvade);
	SAFE_DELETE(m_pCellSpace);
}

void Flock::Update(float deltaT)
{
	// TODO: update the flock
	// loop over all the agents
		// register its neighbors	(-> memory pool is filled with neighbors of the currently evaluated agent)
		// update it				(-> the behaviors can use the neighbors stored in the pool, next iteration they will be the next agent's neighbors)
		// trim it to the world
	for(size_t i = 0; i < m_FlockSize; ++i)
	{
		if(m_Agents[i])
		{
			m_Neighbors.clear();

			if(m_ToggleSpacePart)
			{
				m_pCellSpace->RegisterNeighbors(m_Agents[i], m_NeighborhoodRadius, &m_Neighbors);
				m_NrOfNeighbors = m_Neighbors.size();
				m_pCellSpace->UpdateAgentCell(m_Agents[i], m_Agents[i]->GetOldPosition());
			}
			else
			{
				RegisterNeighbors(m_Agents[i]);
			}

			m_Agents[i]->Update(deltaT);

			if(m_TrimWorld)
			{
				m_Agents[i]->TrimToWorld(m_WorldSize);
			}

			if( i == 0 && m_ToggleDebugRendering)
			{
				for(int neighbor = 0; neighbor < m_NrOfNeighbors; ++neighbor)
				{
					DEBUGRENDERER2D->DrawPoint(m_Neighbors[neighbor]->GetPosition(), 10, { 0,0,1 });
				}
				DEBUGRENDERER2D->DrawCircle(m_Agents[i]->GetPosition(), m_NeighborhoodRadius, { 0,1,1 }, -0.8f);
			}
		}
	}

	m_pAgentToEvade->Update(deltaT);
	m_pAgentToEvade->TrimToWorld(m_WorldSize);

}

void Flock::Render(float deltaT)
{
	// TODO: render the flock
	for(int i = 0; i < m_FlockSize; ++i)
	{
		if(m_Agents[i])
		{
			m_Agents[i]->Render(deltaT);
		}
	}

	m_pAgentToEvade->Render(deltaT);

	if(m_ToggleDebugSpacePart && m_ToggleSpacePart)
	{
		m_pCellSpace->RenderCells();
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

	// TODO: Implement checkboxes for debug rendering and weight sliders here
	ImGui::SliderFloat("Cohesion", GetWeight(m_pCohesionBehavior), 0, 10);
	ImGui::SliderFloat("Separation", GetWeight(m_pSeparationBehavior), 0, 10);
	ImGui::SliderFloat("Velocity Match", GetWeight(m_pVelMatchBehavior), 0, 10);
	ImGui::SliderFloat("Seek", GetWeight(m_pSeekBehavior), 0, 1);
	ImGui::SliderFloat("Wander", GetWeight(m_pWanderBehavior), 0, 1);

	ImGui::Checkbox("Debug rendering", &m_ToggleDebugRendering);
	ImGui::Checkbox("Using space partioning", &m_ToggleSpacePart);
	if(m_ToggleSpacePart)
	{
		ImGui::Checkbox("Draw debug for space partitioning", &m_ToggleDebugSpacePart);
	}

	//End
	ImGui::PopAllowKeyboardFocus();
	ImGui::End();
	
}

void Flock::RegisterNeighbors(SteeringAgent* pAgent)
{
	m_NrOfNeighbors = 0;

	for(int i = 0; i < m_FlockSize; ++i)
	{
		if(m_Agents[i] != pAgent)
		{
			Elite::Vector2 distance{ m_Agents[i]->GetPosition() - pAgent->GetPosition() };
			if(distance.Magnitude() <= m_NeighborhoodRadius)
			{
				m_Neighbors.push_back(m_Agents[i]);
				++m_NrOfNeighbors;
			}
		}
	}
}

Elite::Vector2 Flock::GetAverageNeighborPos() const
{
	Elite::Vector2 avrgPosition{};
	for(size_t i = 0; i < m_Neighbors.size(); ++i)
	{
		avrgPosition += m_Neighbors[i]->GetPosition();
	}
	avrgPosition /= m_NrOfNeighbors;

	return avrgPosition;
}

Elite::Vector2 Flock::GetAverageNeighborVelocity() const
{
	Elite::Vector2 avrgVelocity{};
	for(size_t i = 0; i < m_Neighbors.size(); ++i)
	{
		avrgVelocity += m_Neighbors[i]->GetLinearVelocity();
	}
	avrgVelocity /= m_NrOfNeighbors;

	return avrgVelocity;
}

void Flock::SetTarget_Seek(TargetData target)
{
	// TODO: Set target for seek behavior
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

		if(it!= weightedBehaviors.end())
			return &it->weight;
	}

	return nullptr;
}
