#include "PhysicsEngine.h"

#include <map>
#include <numeric>
#include <unordered_set>

#include "Application.h"
#include "Camera.h"
#include "Force.h"

#include <glm/gtx/matrix_cross_product.hpp>
#include <glm/gtx/orthonormalize.hpp>

using namespace glm;

const glm::vec3 GRAVITY = glm::vec3(0, -9.81, 0);

bool stop = false;

bool CompareEndPoints(Particle::EndPoint a, Particle::EndPoint b)
{
	return a.EndPointValue < b.EndPointValue;
}

void GenerateAndSortAxes(std::vector<Particle>& particles, std::vector<Particle::EndPoint>& endPointsXAxis, std::vector<Particle::EndPoint>& endPointsYAxis, std::vector<Particle::EndPoint>& endPointsZAxis)
{
	// Generate a list of all minEPs and maxEPs for each particle in the list
	for (int i = 0; i < particles.size(); i++)
	{
		endPointsXAxis.insert(endPointsXAxis.end(), std::begin(particles[i].endPointsXAxis), std::end(particles[i].endPointsXAxis));

		endPointsYAxis.insert(endPointsYAxis.end(), std::begin(particles[i].endPointsYAxis), std::end(particles[i].endPointsYAxis));

		endPointsZAxis.insert(endPointsZAxis.end(), std::begin(particles[i].endPointsZAxis), std::end(particles[i].endPointsZAxis));
	}

	std::sort(endPointsXAxis.begin(), endPointsXAxis.end(), CompareEndPoints);
	std::sort(endPointsYAxis.begin(), endPointsYAxis.end(), CompareEndPoints);
	std::sort(endPointsZAxis.begin(), endPointsZAxis.end(), CompareEndPoints);
}

void SymplecticEuler(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{
	vel += accel * dt + impulse / mass;
	pos += vel * dt;
}

void CollisionImpulse(Particle& p, const glm::vec3 cubeCentre, float cubeHalfExtent, float coeffOfRestitution)
{
	vec3 impulse{ 0.0f };
	vec3 surfaceNorm{ 0.0f };

	for (int i = 0; i < 3; i++)
	{
		if (p.Position()[i] + p.Scale()[i] >= (cubeCentre[i] + cubeHalfExtent))
		{
			surfaceNorm = vec3(0.0f);
			surfaceNorm[i] = -1.0f;
			if (i == 0)
			{
				p.SetPosition(vec3(cubeHalfExtent - p.Scale()[i], p.Position().y, p.Position().z));
			}
			else if (i == 1)
			{
				p.SetPosition(vec3(p.Position().x, cubeHalfExtent - p.Scale()[i], p.Position().z));
			}
			else if (i == 2)
			{
				p.SetPosition(vec3(p.Position().x, p.Position().y, cubeHalfExtent - p.Scale()[i]));
			}
		}
		else if (p.Position()[i] - p.Scale()[i] <= (cubeCentre[i] - cubeHalfExtent))
		{
			surfaceNorm = vec3(0.0f);
			surfaceNorm[i] = 1.0f;

			if (i == 0)
			{
				p.SetPosition(vec3(-cubeHalfExtent + p.Scale()[i], p.Position().y, p.Position().z));
			}
			else if (i == 1)
			{
				p.SetPosition(vec3(p.Position().x, -cubeHalfExtent + p.Scale()[i], p.Position().z));
			}
			else if (i == 2)
			{
				p.SetPosition(vec3(p.Position().x, p.Position().y, -cubeHalfExtent + p.Scale()[i]));
			}
		}
	}

	impulse = -(1.0f + coeffOfRestitution) * p.Mass() * glm::dot(p.Velocity(), surfaceNorm) * surfaceNorm;
	p.SetVelocity(p.Velocity() + impulse / p.Mass());
}

bool DetectCollisionBetweenSpheres(Particle& p1, Particle& p2)
{
	// Squared distance between centers
	float distance = (p1.Position().x - p2.Position().x) * (p1.Position().x - p2.Position().x) +
		(p1.Position().y - p2.Position().y) * (p1.Position().y - p2.Position().y) +
		(p1.Position().z - p2.Position().z) * (p1.Position().z - p2.Position().z);

	float radiusSum = p1.Scale().x + p2.Scale().x;

	return distance <= radiusSum * radiusSum;
}

void ResolveStaticCollision(Particle& p1, Particle& p2)
{
	float distance = glm::distance(p1.Position(), p2.Position());

	float overlap = 0.5f * (distance - p1.Scale().x - p2.Scale().y);


	vec3 dir = glm::normalize(p1.Position() - p2.Position());
	float newPosX = overlap * dir.x;
	float newPosY = overlap * dir.y;
	float newPosZ = overlap * dir.z;


	p1.SetPosition(vec3(p1.Position().x - newPosX, p1.Position().y - newPosY, p1.Position().z - newPosZ));
	p2.SetPosition(vec3(p2.Position().x + newPosX, p2.Position().y + newPosY, p2.Position().z + newPosZ));
}

// This is called once
void PhysicsEngine::Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb)
{
	// Get a few meshes/shaders from the databases
	auto defaultShader = shaderDb.Get("default");
	auto groundMesh = meshDb.Get("plane");

	meshDb.Add("cube", Mesh(MeshDataFromWavefrontObj("resources/models/cube.obj")));
	meshDb.Add("sphere", Mesh(MeshDataFromWavefrontObj("resources/models/sphere.obj")));
	meshDb.Add("cone", Mesh(MeshDataFromWavefrontObj("resources/models/cone.obj")));

	// Initialise ground
	ground.SetMesh(meshDb.Get("cube"));
	ground.SetShader(defaultShader);
	ground.SetScale(vec3(30.0f));
	for (int i = 0; i < 3; i++)
	{
		Particle p;
		p.SetMesh(meshDb.Get("sphere"));
		p.SetShader(defaultShader);

		p.SetColor(vec4(1, 0, 0, 1));
		p.SetScale(vec3(1.0f));

		p.SetPosition(vec3(-30 + (rand() % 30), -30 + (rand() % 30), -30 + (rand() % 30)));

		particles.push_back(p);
	}

	particles[0].SetScale(vec3(1.0f));
	particles[1].SetScale(vec3(2.0f));
	particles[2].SetScale(vec3(3.0f));

	particles[0].SetPosition(vec3(0.0f, 0.0f, 0.0f));
	particles[1].SetPosition(vec3(5.0f, 0.0f, 0.0f));
	particles[2].SetPosition(vec3(-5.0f, 0.0f, 0.0f));

	particles[0].SetVelocity(vec3(0.0f, 0.0f, 0.0f));
	particles[1].SetVelocity(vec3(-20.0f, 0.0f, 0.0f));
	particles[2].SetVelocity(vec3(20.0f, 0.0f, 0.0f));





	camera = Camera(vec3(0, 5, 10));
}

// This is called every frame
void PhysicsEngine::Update(float deltaTime, float totalTime)
{
	if(true)
	{
		for (int i = 0; i < particles.size(); i++)
		{
			particles[i].ClearForcesImpulses();


			Force::Gravity(particles[i]);

			vec3 acceleration = particles[i].AccumulatedForce() / particles[i].Mass();

			vec3 position = particles[i].Position();
			vec3 velocity = particles[i].Velocity();
			SymplecticEuler(position, velocity, particles[i].Mass(), acceleration, particles[i].AccumulatedImpulse(), deltaTime);

			particles[i].SetPosition(position);
			particles[i].SetVelocity(velocity);

		}

		std::vector<Particle::EndPoint> endPointsXAxis;
		std::vector<Particle::EndPoint> endPointsYAxis;
		std::vector<Particle::EndPoint> endPointsZAxis;

		GenerateAndSortAxes(particles, endPointsXAxis, endPointsYAxis, endPointsZAxis);
		// Implementing check for each axis
		// Get a vector of pairs.
		std::vector<std::pair<Particle*, Particle*>> vecOfPairs;
		// Start from first axis, check for overlapping projections, if overlap, add to pairs.
		for (int i = 0; i < endPointsXAxis.size() - 1; i++)
		{
			// For each object in the list, find its min on X List
			if (endPointsXAxis[i].isMin)
			{
				PhysicsBody* currObj = endPointsXAxis[i].Owner;
				int k = 1;
				bool shouldPairY = false;
				bool shouldPairZ = false;

				while(endPointsXAxis[i+k].isMin || (!endPointsXAxis[i+k].isMin && endPointsXAxis[i].Owner != endPointsXAxis[i+k].Owner))
				{

					// Y Axis check
					for(int y = 0; y < endPointsYAxis.size() - 1; y++)
					{
						if(currObj == endPointsYAxis[y].Owner && endPointsYAxis[y].isMin)
						{
							int a = 1;

							while(endPointsYAxis[y+a].isMin || (!endPointsYAxis[y + a].isMin && endPointsYAxis[y].Owner != endPointsYAxis[y + a].Owner))
							{
								if(endPointsXAxis[i+k].Owner == endPointsYAxis[y+a].Owner)
								{
									shouldPairY = true;
									break;
								}
								else
								{
									shouldPairY = false;
								}
								a++;
							}


						}
						else if(shouldPairY)
						{
							break;
						}

					}

					// Z Axis check
					if(shouldPairY)
					{
						for (int z = 0; z < endPointsZAxis.size() - 1; z++)
						{
							if (currObj == endPointsZAxis[z].Owner && endPointsZAxis[z].isMin)
							{
								int b = 1;

								while (endPointsZAxis[z + b].isMin || (!endPointsZAxis[z + b].isMin && endPointsZAxis[z].Owner != endPointsZAxis[z + b].Owner))
								{
									if (endPointsXAxis[i + k].Owner == endPointsZAxis[z + b].Owner)
									{
										shouldPairZ = true;
										break;
									}
									else
									{
										shouldPairZ = false;
									}
									b++;
								}

								
							}
							else if (shouldPairZ)
							{
								break;
							}
						}
					}


					if(shouldPairY && shouldPairZ)
					{
							std::pair<Particle*, Particle*> newPair;
							newPair.first = endPointsXAxis[i].Owner;
							newPair.second = endPointsXAxis[i + k].Owner;

							vecOfPairs.push_back(newPair);
							stop = true;
					}
					k++;
				}

				//i += k - 1;
				
				//int k = 1;
				//while (endPointsXAxis[i + k].isMin)
				//{
				//	std::pair<PhysicsBody*, PhysicsBody*> newPair;
				//	newPair.first = endPointsXAxis[i].Owner;
				//	newPair.second = endPointsXAxis[i + k].Owner;

				//	vecOfPairs.push_back(newPair);
				//	k++;
				//}
				//i += k - 1;
				//stop = true;
			}
		}

		
		for (int i = 0; i < particles.size(); i++)
		{
			CollisionImpulse(particles[i], vec3(0.0f), 30.0f, 0.85f);

			for(int k = 0; k < vecOfPairs.size(); k++)
			{
				if(DetectCollisionBetweenSpheres(*vecOfPairs[k].first, *vecOfPairs[k].second))
				{
					std::cout << "number of collisions : " << vecOfPairs.size() << std::endl;
					if(vecOfPairs.size() == 2)
					{
						std::cout << "Stop" << std::endl;
					}
					ResolveStaticCollision(*vecOfPairs[k].first, *vecOfPairs[k].second);

					vec3 normal = glm::normalize(vecOfPairs[k].first->Position() - vecOfPairs[k].second->Position());

					float meff = 1 / ((1 / vecOfPairs[k].first->Mass()) + (1 / vecOfPairs[k].second->Mass()));

					float impactSpeed = glm::dot(normal, (vecOfPairs[k].first->Velocity() - vecOfPairs[k].second->Velocity()));

					float impulse = (1 + 0.85) * meff * impactSpeed;

					vec3 dVel1 = -(impulse / vecOfPairs[k].first->Mass() * normal);
					vec3 dVel2 = +(impulse / vecOfPairs[k].second->Mass() * normal);

					vecOfPairs[k].first->SetVelocity(vecOfPairs[k].first->Velocity() + dVel1);
					vecOfPairs[k].second->SetVelocity(vecOfPairs[k].second->Velocity() + dVel2);
				}
			}
			//for (int j = i + 1; j < particles.size(); j++)
			//{
			//	if (DetectCollisionBetweenSpheres(particles[i], particles[j]))
			//	{
			//		counterOfCollision++;
			//		ResolveStaticCollision(particles[i], particles[j]);

			//		vec3 normal = glm::normalize(particles[j].Position() - particles[i].Position());

			//		float meff = 1 / ((1 / particles[i].Mass()) + (1 / particles[j].Mass()));

			//		float impactSpeed = glm::dot(normal, (particles[i].Velocity() - particles[j].Velocity()));

			//		float impulse = (1 + 0.85) * meff * impactSpeed;

			//		vec3 dVel1 = -(impulse / particles[i].Mass() * normal);
			//		vec3 dVel2 = +(impulse / particles[j].Mass() * normal);

			//		particles[i].SetVelocity(particles[i].Velocity() + dVel1);
			//		particles[j].SetVelocity(particles[j].Velocity() + dVel2);
			//	}
			//}

		}

	}







}

// This is called every frame, after Update
void PhysicsEngine::Display(const mat4& viewMatrix, const mat4& projMatrix)
{
	ground.Draw(viewMatrix, projMatrix);
	for (auto& p : particles)
		p.Draw(viewMatrix, projMatrix);
}

void PhysicsEngine::HandleInputKey(int keyCode, bool pressed)
{
	switch (keyCode)
	{
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		// TODO: Add any task swapping keys here
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	default:
		break;
	}
}