#include "RigidBody.h"

void RigidBody::add_force(Vec3 force, Vec3 world_location)
{
	Mat4 translation_matrix;
	translation_matrix.initTranslation(-this->center_of_mass.x, -this->center_of_mass.y, -this->center_of_mass.z);

	this->forces += force;
	Vec3 temp_loc = translation_matrix.transformVector(world_location);
	Vec3 temp_force = force;
	this->torque += cross(temp_loc, temp_force);
}

std::vector<Vec3> RigidBody::get_world_points()
{
	std::vector<Vec3> world_points;
	Mat4 rotational_matrix = this->orientation.getRotMat();
	
	for (Vec3 point : this->body_points) {
		world_points.emplace_back(rotational_matrix.transformVector(point));
	}

	return world_points;
}

RigidBody RigidBody::makeQuad(Vec3 position, Vec3 size, int mass)
{
	RigidBody body;

	float x = 0.5;
	float y = 0.5;
	float z = 0.5;

	body.body_points = std::vector<Vec3>{
		Vec3( x,  y,  z),
		Vec3(-x,  y,  z),
		Vec3( x, -y,  z),
		Vec3(-x, -y,  z),
		Vec3( x,  y, -z),
		Vec3(-x,  y, -z),
		Vec3( x, -y, -z),
		Vec3(-x, -y, -z)
	};

	body.scale_matrix.initScaling(size.x, size.y, size.z);
	body.center_of_mass = position;
	body.orientation = Quat(0, 0, 0, 1);

	body.forces = 0;
	body.torque = 0;
	body.linear_velocity = 0;
	body.angular_velocity = 0;
	body.angular_momentum = 0;
	body.mass = mass;

	body.inverse_base_inertia_tensor = (Mat4(
		(size.y * size.y + size.z * size.z), 0, 0, 0,
		0, (size.x * size.x + size.z * size.z), 0, 0,
		0, 0, (size.x * size.x + size.y * size.y), 0,
		0, 0,                                   0, (12./mass)
	) * (mass/12.)).inverse();

	return body;
}

Mat4 RigidBody::getBodyToWorld()
{
	Mat4 translation_matrix;
	translation_matrix.initTranslation(this->center_of_mass.x, this->center_of_mass.y, this->center_of_mass.z);

	return this->scale_matrix * this->orientation.getRotMat() * translation_matrix;
}

Mat4 RigidBody::getWorldToBody()
{
	return getBodyToWorld().inverse();
}

Vec3 RigidBody::getWorldVelocityAt(const Vec3& world_location)
{
	Vec3 body_location = getWorldToBody().transformVector(world_location);

	return this->linear_velocity + cross(this->angular_velocity, body_location);
}

Vec3 RigidBody::calcImpulse(const Vec3& world_location, const Vec3& world_normal)
{
	Vec3 body_location = getWorldToBody().transformVector(world_location);

	Mat4 rotational_matrix = this->orientation.getRotMat();
	Mat4 inverse_rotational_matrix = this->orientation.getRotMat();
	inverse_rotational_matrix.transpose();
	Mat4 inverse_inertia_tensor = rotational_matrix * this->inverse_base_inertia_tensor * inverse_rotational_matrix;

	return cross(inverse_inertia_tensor.transformVector(cross(body_location, world_normal)), body_location);
}

void RigidBody::print()
{
	printf("CoM: %f %f %f\n", this->center_of_mass.x, this->center_of_mass.y, this->center_of_mass.z);
	printf("v_lin: %f %f %f\n", this->linear_velocity.x, this->linear_velocity.y, this->linear_velocity.z);
	printf("v_ang: %f %f %f\n", this->angular_velocity.x, this->angular_velocity.y, this->angular_velocity.z);
	Vec3 body_location = getWorldToBody().transformVector(Vec3(0.3,0.5,0.25));
	Vec3 result = this->linear_velocity + cross(this->angular_velocity, body_location);
	printf("v of  (0.3, 0.5, 0.25): %f %f %f\n",result.x, result.y, result.z);
}

void RigidBody::do_step(float h)
{
	this->center_of_mass += h * this->linear_velocity;
	this->linear_velocity += h * (this->forces / this->mass);

	Quat temp = Quat(this->angular_velocity[0], this->angular_velocity[1], this->angular_velocity[2], 0);

	this->orientation = (this->orientation + (h / 2) * temp * this->orientation).unit();

	this->angular_momentum += h * this->torque;

	Mat4 rotational_matrix = this->orientation.getRotMat();
	Mat4 inverse_rotational_matrix = this->orientation.getRotMat();
	inverse_rotational_matrix.transpose();
	Mat4 inverse_inertia_tensor = rotational_matrix * this->inverse_base_inertia_tensor * inverse_rotational_matrix;
	this->angular_velocity = inverse_inertia_tensor.transformVector(this->angular_momentum);

	this->forces = 0;
	this->torque = 0;
}
