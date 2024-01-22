#ifndef RIGIDBODY_H
#define RIGIDBODY_H
#include "Simulator.h"

class RigidBody {

public:
	RigidBody() = default;

	Vec3 center_of_mass;
	Vec3 linear_velocity;
	Vec3 angular_velocity;
	Vec3 angular_momentum;
	Quat orientation;
	float mass;

	void add_force(Vec3 force, Vec3 world_location);
	std::vector<Vec3> get_world_points();

	void do_step(float h);

	static RigidBody makeQuad(Vec3 position, Vec3 size, int mass);
	Mat4 getBodyToWorld();
	Mat4 getWorldToBody();

	Vec3 getWorldVelocityAt(const Vec3& world_location);
	Vec3 calcImpulse(const Vec3& world_location, const Vec3& world_normal);

	void print();

private:

	Mat4 inverse_base_inertia_tensor;
	std::vector<Vec3> body_points;
	Vec3 forces;
	Vec3 torque;
	Mat4 scale_matrix;

};
#endif // RIGIDBODY_H
