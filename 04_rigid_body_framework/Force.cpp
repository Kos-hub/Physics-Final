#include "PhysicsEngine.h"
#include "Force.h"
#include "PhysicsObject.h"

using namespace glm;
const float AIR_DENSITY = 1.225f;
const float DRAG_COEFF = 0.47f;

void Force::Gravity(Particle& p)
{
	auto force = vec3(0, -9.81, 0) * p.Mass();
	p.ApplyForce(force);
}

void Force::Drag(Particle& p)
{
	// Each particle has the same area (using the Symp for getting the scale value)
	float area = glm::pi<float>() * p.Scale().x * p.Scale().x;

	// AeroForce calculation. Null checking the velocity so that It doesn't return NaN
	vec3 aeroForce = vec3(0.0f);

	if (glm::length(p.Velocity()) != 0.0f)
	{
		aeroForce = 0.5f * AIR_DENSITY *
			(glm::length(p.Velocity()) * glm::length(p.Velocity())) *
			DRAG_COEFF * area *
			(-glm::normalize(p.Velocity()));
	}

	p.ApplyForce(aeroForce);
}

void Force::Hooke(Particle& p1, Particle& p2, float restLength, float ks, float kd)
{
	// Compute distance between P1 and P2
	float distance = glm::distance(p2.Position(), p1.Position());

	// Compute the unit vector e from P1 and P2
	vec3 unitVector1 = glm::normalize(p2.Position() - p1.Position());
	vec3 unitVector2 = glm::normalize(p1.Position() - p2.Position());

	// Compute 1D velocities with dot prod
	float p1Vel = glm::dot(unitVector1, p1.Velocity());
	float p2Vel = glm::dot(unitVector2, p2.Velocity());

	// Back to 3D with Fsd force calculation
	float fsd1 = -ks * (restLength - distance) - kd * p1Vel;
	float fsd2 = -ks * (restLength - distance) - kd * p2Vel;

	p1.ApplyForce(fsd1 * unitVector1);
	p2.ApplyForce(fsd2 * unitVector2);
}