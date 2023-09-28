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
int sortAxis = 0;
MeshDb* tempMeshDb;
ShaderDb* tempShaderDb;
const float COEFF_OF_RESTITUTION = 0.85f;


// Helper function for comparing Particles
bool compareParticles(Particle& p1, Particle& p2)
{
	return p1.minEndPoints[sortAxis] < p2.minEndPoints[sortAxis];
}

// Symplectic integration
void SymplecticEuler(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{
	vel += accel * dt;
	pos += vel * dt;
}

// Collisions between boundaries
void CollisionImpulse(Particle& p, const glm::vec3 cubeCentre, float cubeHalfExtent, float coeffOfRestitution)
{
	vec3 impulse{ 0.0f };
	vec3 surfaceNorm{ 0.0f };

	for (int i = 0; i < 3; i++)
	{
		if (p.Position()[i] + p.Scale()[i] >= (cubeCentre[i] + cubeHalfExtent))
		{
			//surfaceNorm = vec3(0.0f);
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
			//surfaceNorm = vec3(0.0f);
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

// Detecting collisions between spheres.
bool DetectCollisionBetweenSpheres(Particle& p1, Particle& p2)
{
	// Squared distance between centers
	float distance = (p1.Position().x - p2.Position().x) * (p1.Position().x - p2.Position().x) +
		(p1.Position().y - p2.Position().y) * (p1.Position().y - p2.Position().y) +
		(p1.Position().z - p2.Position().z) * (p1.Position().z - p2.Position().z);

	float radiusSum = p1.Scale().x + p2.Scale().x;

	return distance <= radiusSum * radiusSum;
}

// Shifting the Spheres after detecting collision.
void ResolveStaticCollision(Particle& p1, Particle& p2)
{
	float distance = glm::distance(p1.Position(), p2.Position());

	if (distance == 0.0f)
	{
		p1.SetPosition(vec3(p1.Position().x + 0.1f, p1.Position().y, p1.Position().z));
		distance = glm::distance(p1.Position(), p2.Position());
	}
	
	float overlap = 0.5f * (distance - p1.Scale().x - p2.Scale().x);

	vec3 dir = glm::normalize(p1.Position() - p2.Position());

	float newPosX = overlap * dir.x;
	float newPosY = overlap * dir.y;
	float newPosZ = overlap * dir.z;


	p1.SetPosition(vec3(p1.Position().x - newPosX, p1.Position().y - newPosY, p1.Position().z - newPosZ));
	p2.SetPosition(vec3(p2.Position().x + newPosX, p2.Position().y + newPosY, p2.Position().z + newPosZ));
}

// Calculating the impulse between spheres.
void CalculateImpulseBetweenSpheres(Particle& p1, Particle& p2)
{
	vec3 normal = glm::normalize(p2.Position() - p1.Position());

	float meff = 1 / ((1 / p1.Mass()) + (1 / p2.Mass()));

	float impactSpeed = glm::dot(normal, (p2.Velocity() - p1.Velocity()));

	float impulse = (1 + COEFF_OF_RESTITUTION) * meff * impactSpeed;

	vec3 dVel1 = +(impulse / p1.Mass() * normal);
	vec3 dVel2 = -(impulse / p2.Mass() * normal);

	p1.SetVelocity(p1.Velocity() + dVel1);
	p2.SetVelocity(p2.Velocity() + dVel2);
}

// Function that adds a random sphere in a random position.
void PhysicsEngine::AddRandomSphere()
{
	Particle randPart;
	randPart.SetMesh(tempMeshDb->Get("sphere"));
	randPart.SetShader(tempShaderDb->Get("default"));

	int whichRGB = rand() % 3;
	vec4 color = vec4(0, 0, 0, 1);
	color[whichRGB] = 1;

	if (whichRGB == 0)
	{
		randPart.SetMass(1);
		randPart.SetScale(vec3(1.0f));
	}
	if (whichRGB == 1)
	{
		randPart.SetMass(2);
		randPart.SetScale(vec3(2.0f));
	}

	if (whichRGB == 2)
	{
		randPart.SetMass(3);
		randPart.SetScale(vec3(3.0f));
	}

	randPart.SetColor(color);

	randPart.SetPosition(vec3(-30 + (rand() % 59), -15 + (rand() % 29), -30 + (rand() % 59)));

	randPart.SetVelocity(vec3(-20 + rand() % 39, -20 + rand() % 39, -20 + rand() % 39));
	particles.push_back(randPart);

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

	tempMeshDb = &meshDb;
	tempShaderDb = &shaderDb;



	// Initialise ground
	ground.SetMesh(meshDb.Get("cube"));
	ground.SetShader(defaultShader);
	ground.SetScale(vec3(30.0f));
	ground.SetPosition(vec3(ground.Position().x, -30.0f * 2.0f, ground.Position().z));



	srand(1);
	for (int i = 0; i < 200; i++)
	{
		Particle p;
		p.SetMesh(meshDb.Get("sphere"));
		p.SetShader(defaultShader);

		// Getting a random value between 0 and 3
		int whichRGB = rand() % 3;
		vec4 color = vec4(0, 0, 0, 1);

		// That random value will be either red, green or blue
		color[whichRGB] = 1;

		// If sphere is red, radius is 1 and mass is 1; if sphere is blue, radius is 2 and mass is 1; if sphere is green, radius is 3 and mass is 3.
		if(whichRGB == 0)
		{
			p.SetMass(1);
			p.SetScale(vec3(1.0f));
		}
		if (whichRGB == 1)
		{
			p.SetMass(2);
			p.SetScale(vec3(2.0f));
		}
		if (whichRGB == 2)
		{
			p.SetMass(3);
			p.SetScale(vec3(3.0f));
		}


		p.SetColor(color);
		
		p.SetPosition(vec3( -30 + (rand() % 59), -30 + (rand() % 59), -30 + (rand() % 59)));

		p.SetVelocity(vec3(-20 + rand() % 39, -20 + rand() % 39, -20 + rand() % 39));
		particles.push_back(p);
	}


	camera = Camera(vec3(0, 5, 30));
}

// This is called every frame
void PhysicsEngine::Update(float deltaTime, float totalTime)
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

		
		CollisionImpulse(particles[i], vec3(0.0f), 30.0f, COEFF_OF_RESTITUTION);

	}

	vec3 s = vec3(0.0f), s2 = vec3(0.0f), v;

	// Sorting spheres
	std::sort(particles.begin(), particles.end(), compareParticles);

	for (int i = 0; i < particles.size(); i++)
	{
		// Variance calculations
		for(int c = 0; c < 3; c++)
		{
			s[c] += particles[i].Position()[c];
			s2[c] += particles[i].Position()[c] * particles[i].Position()[c];
		}

		for(int j = i + 1; j < particles.size(); j++)
		{
			if (particles[j].minEndPoints[sortAxis] > particles[i].maxEndPoints[sortAxis])
				break;
			if (particles[i].maxEndPoints[sortAxis] >= particles[j].minEndPoints[sortAxis])
			{
				if(DetectCollisionBetweenSpheres(particles[i], particles[j]))
				{
					ResolveStaticCollision(particles[i], particles[j]);

					CalculateImpulseBetweenSpheres(particles[i], particles[j]);
				}

			}

		}
	}

	// Variance calculations
	for (int c = 0; c < 3; c++)
		v[c] = s2[c] - s[c] * s[c] / particles.size();


	// Picking one axis based on the variance
	sortAxis = 0;
	if (v[1] > v[0]) sortAxis = 1;
	if (v[2] > v[sortAxis]) sortAxis = 2;

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
	case GLFW_KEY_SPACE:
		if (pressed)
			AddRandomSphere();
	default:
		break;
	}
}