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

int sortAxis = 0;
//bool CompareEndPoints(Particle::EndPoint a, Particle::EndPoint b)
//{
//	return a.EndPointValue < b.EndPointValue;
//}
//
//void GenerateAndSortAxes(std::vector<Particle>& particles, std::vector<Particle::EndPoint>& endPointsXAxis, std::vector<Particle::EndPoint>& endPointsYAxis, std::vector<Particle::EndPoint>& endPointsZAxis)
//{
//	// Generate a list of all minEPs and maxEPs for each particle in the list
//	for (int i = 0; i < particles.size(); i++)
//	{
//		endPointsXAxis.insert(endPointsXAxis.end(), std::begin(particles[i].endPointsXAxis), std::end(particles[i].endPointsXAxis));
//
//		endPointsYAxis.insert(endPointsYAxis.end(), std::begin(particles[i].endPointsYAxis), std::end(particles[i].endPointsYAxis));
//
//		endPointsZAxis.insert(endPointsZAxis.end(), std::begin(particles[i].endPointsZAxis), std::end(particles[i].endPointsZAxis));
//	}
//
//	std::sort(endPointsXAxis.begin(), endPointsXAxis.end(), CompareEndPoints);
//	std::sort(endPointsYAxis.begin(), endPointsYAxis.end(), CompareEndPoints);
//	std::sort(endPointsZAxis.begin(), endPointsZAxis.end(), CompareEndPoints);
//}

bool compareParticles(Particle& p1, Particle& p2)
{
	return p1.minEndPoints[sortAxis] < p2.minEndPoints[sortAxis];
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
	for (int i = 0; i < 100; i++)
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

			CollisionImpulse(particles[i], vec3(0.0f), 30.0f, 0.85f);

		}

		vec3 s = vec3(0.0f), s2 = vec3(0.0f), v;

		std::sort(particles.begin(), particles.end(), compareParticles);

		// TODO: Create functions to clear up the code.
		for (int i = 0; i < particles.size(); i++)
		{
			for(int c = 0; c < 3; c++)
			{
				s[c] += particles[i].Position()[c];
				s2[c] += particles[i].Position()[c] * particles[i].Position()[c];
			}

			for(int j = i + 1; j < particles.size(); j++)
			{
				if (particles[j].minEndPoints[sortAxis] > particles[i].maxEndPoints[sortAxis])
					break;
				if (particles[i].maxEndPoints[sortAxis] > particles[j].minEndPoints[sortAxis])
				{
					if(DetectCollisionBetweenSpheres(particles[i], particles[j]))
					{
						ResolveStaticCollision(particles[i], particles[j]);

						vec3 normal = glm::normalize(particles[j].Position() - particles[i].Position());

						float meff = 1 / ((1 / particles[i].Mass()) + (1 / particles[j].Mass()));

						float impactSpeed = glm::dot(normal, (particles[i].Velocity() - particles[j].Velocity()));

						float impulse = (1 + 0.85) * meff * impactSpeed;

						vec3 dVel1 = -(impulse / particles[i].Mass() * normal);
						vec3 dVel2 = +(impulse / particles[j].Mass() * normal);

						particles[i].SetVelocity(particles[i].Velocity() + dVel1);
						particles[j].SetVelocity(particles[j].Velocity() + dVel2);
					}

				}

			}
		}

		for (int c = 0; c < 3; c++)
			v[c] = s2[c] - s[c] * s[c] / particles.size();


		sortAxis = 0;
		if (v[1] > v[0]) sortAxis = 1;
		if (v[2] > v[sortAxis]) sortAxis = 2;

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