#include "PhysicsObject.h"

#include <glm/glm.hpp>

#include "PhysicsEngine.h"
#include "Mesh.h"
#include "Shader.h"


void PhysicsBody::Draw(const glm::mat4& viewMatrix, const glm::mat4& projectionMatrix) const
{
	m_shader->Use();
	//m_shader->SetUniform("model", ModelMatrix());
	//m_shader->SetUniform("view", viewMatrix);
	//m_shader->SetUniform("projection", projectionMatrix);
	m_shader->SetUniform("color", m_color);

	auto mvp = projectionMatrix * viewMatrix * ModelMatrix();
	m_shader->SetUniform("modelViewProjectionMatrix", mvp);
	m_shader->SetUniform("normalMatrix", transpose(inverse(viewMatrix * ModelMatrix())));
	m_mesh->DrawVertexArray();
}

void RigidBody::SetScale(const glm::vec3& scale)
{
	Particle::SetScale(scale);

	SetInertiaTensor();
}

void RigidBody::SetMass(float mass)
{
	Particle::SetMass(mass);

	SetInertiaTensor();
}

glm::mat3 RigidBody::InverseInertia()
{
	return this->Orientation() * glm::mat4(glm::inverse(m_inertiaTensor)) * glm::transpose(this->Orientation());
}

glm::mat3 RigidBody::Inertia()
{
	return this->Orientation() * glm::mat4(m_inertiaTensor) * glm::transpose(this->Orientation());
}


void RigidBody::SetInertiaTensor()
{
	glm::mat3 inertiaTensor(1.0f);
	inertiaTensor[0][0] = (1.0f / 12.0f) * this->Mass() * ((this->Scale().y * 2) * (this->Scale().y * 2) + (this->Scale().z * 2) * (this->Scale().z * 2));
	inertiaTensor[1][1] = (1.0f / 12.0f) * this->Mass() * ((this->Scale().x * 2) * (this->Scale().x * 2) + (this->Scale().z * 2) * (this->Scale().z * 2));
	inertiaTensor[2][2] = (1.0f / 12.0f) * this->Mass() * ((this->Scale().x * 2) * (this->Scale().x * 2) + (this->Scale().y * 2) * (this->Scale().y * 2));


	m_inertiaTensor = inertiaTensor;
}
