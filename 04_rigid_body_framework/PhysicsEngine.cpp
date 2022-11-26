#include "PhysicsEngine.h"

#include <map>
#include <numeric>

#include "Application.h"
#include "Camera.h"
#include "Force.h"

#include <glm/gtx/matrix_cross_product.hpp>
#include <glm/gtx/orthonormalize.hpp>

using namespace glm;

const glm::vec3 GRAVITY = glm::vec3(0, -9.81, 0);

bool stop = false;
void SymplecticEuler(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{
	vel += accel * dt + impulse / mass;
	pos += vel * dt;
}

vec3 CollisionImpulse(Particle& p, const glm::vec3 cubeCentre, float cubeHalfExtent, float coeffOfRestitution)
{
	vec3 impulse{ 0.0f };
	vec3 surfaceNorm{ 0.0f };

	for(int i = 0; i < 3; i++)
	{
		if(p.Position()[i] + p.Scale()[i] >= (cubeCentre[i] + cubeHalfExtent))
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
	return impulse;
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
	for(int i = 0; i < 2; i++)
	{
		Particle p;
		p.SetMesh(meshDb.Get("sphere"));
		p.SetShader(defaultShader);

		p.SetColor(vec4(1, 0, 0, 1));
		p.SetScale(vec3(1.0f));


		particles.push_back(p);
	}

	particles[0].SetPosition(vec3(5.0f, 0.0f, 0.0f));
	particles[1].SetPosition(vec3(-5.0f, 0.0f, 0.0f));

	particles[0].SetVelocity(vec3(-20.0f, 0.0f, 0.0f));
	particles[1].SetVelocity(vec3(20.0f, 0.0f, 0.0f));

	particles[1].SetScale(vec3(0.5f));
	camera = Camera(vec3(0, 5, 10));
}

// This is called every frame
void PhysicsEngine::Update(float deltaTime, float totalTime)
{

	if(!stop)
	{
		for (int i = 0; i < particles.size(); i++)
		{
			particles[i].ClearForcesImpulses();
			particles[i].ApplyImpulse(CollisionImpulse(particles[i], vec3(0.0f), 30.0f, 0.85f));

			for (int j = i + 1; j < particles.size(); j++)
			{
				if (DetectCollisionBetweenSpheres(particles[i], particles[j]))
				{
					ResolveStaticCollision(particles[i], particles[j]);
				}
			}

			Force::Gravity(particles[i]);

			vec3 acceleration = particles[i].AccumulatedForce() / particles[i].Mass();

			vec3 position = particles[i].Position();
			vec3 velocity = particles[i].Velocity();
			SymplecticEuler(position, velocity, particles[i].Mass(), acceleration, particles[i].AccumulatedImpulse(), deltaTime);

			particles[i].SetPosition(position);
			particles[i].SetVelocity(velocity);

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