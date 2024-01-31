#include "RigidBodySystemSimulator.h"
#include "RigidBody.h"
#include "collisionDetect.h"

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	this->m_iTestCase = 0;
	this->m_massSpringSystem = MassSpringSystemSimulator();
	//notifyCaseChanged(this->m_iTestCase);
}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
	return "Demo1,Demo2,Demo3";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	this->m_massSpringSystem.initUI(DUC);
}

void RigidBodySystemSimulator::reset()
{
	this->m_massSpringSystem.reset();
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	this->m_massSpringSystem.drawFrame(pd3dImmediateContext);

	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1), 2000.0, Vec3(0.5));

	for (RigidBody& body : bodies)
	{
		DUC->drawRigidBody(body.getBodyToWorld());
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	this->m_massSpringSystem.notifyCaseChanged(testCase);

	this->m_iTestCase = testCase;
	bodies.clear();

	switch (testCase)
	{
	case 0:
	{
		//construct goal
		//addRigidBody(Vec3(1, 0, -0.5), Vec3(0.1, 1, 0.1), 2);
		//addRigidBody(Vec3(1, 0, 0.5), Vec3(0.1, 1, 0.1), 2);
		//applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));

		//construct ball
		addRigidBody(Vec3(0, 0, 0), Vec3(0.3, 0.3, 0.3), 2, true);
		break;
	}
	case 1:
	{		
		//construct goal
		addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.3), 2, true);
		setOrientationOf(0, Quat(Vec3(0, 0, 1), M_PI_2));
		applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));
	}

	case 2:
	{
		//goal
		addRigidBody(Vec3(-1, 0.75, 0), Vec3(0.1, 1.5, 0.1), 10000, false);
		addRigidBody(Vec3(1, 0.75, 0), Vec3(0.1, 1.5, 0.1), 10000, false);
		addRigidBody(Vec3(0, 1.5, 0), Vec3(2.1, 0.1, 0.1), 10000, false);


		addRigidBody(Vec3(0, 0.5, -2), Vec3(0.3, 0.3, 0.3), 2, true);
	}
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
	if (this->m_iTestCase != 3)
		return;

	Vec3 gravity(0, -0.981, 0);

	for (int i = 0;i < bodies.size();i++)
	{
		RigidBody& body = bodies[i];
		Vec3 loc = body.center_of_mass + Vec3(0,0.5,0);
		applyForceOnBody(i, loc, gravity * body.mass * timeElapsed);
	}
}

ostream& operator<<(ostream& os, const RigidBody& body)
{
	os << "RigidBody at " << body.center_of_mass << " with linear velocity " << body.linear_velocity << ", angular velocity " << body.angular_velocity << " and mass " << body.mass;
	return os;
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	timeStep *= 10;
	this->m_massSpringSystem.simulateTimestep(timeStep);

	for (RigidBody& body : bodies)
	{
		body.do_step(timeStep);
	}

	handleCollisionAmongBodies();
}


void RigidBodySystemSimulator::handleCollisionAmongBodies()
{
	for (int i = 0;i < bodies.size();i++)
	{
		RigidBody& body_a = bodies[i];
		handleCollisionWithMSS(body_a);
		for (int j = 0;j < bodies.size();j++)
		{
			RigidBody& body_b = bodies[j];

			CollisionInfo info = checkCollisionSAT(body_a.getBodyToWorld(), body_b.getBodyToWorld());

			if (!info.isValid) // Bodies are not colliding: do nothing
				continue;

			Vec3 vel_a = body_a.getWorldVelocityAt(info.collisionPointWorld);
			Vec3 vel_b = body_b.getWorldVelocityAt(info.collisionPointWorld);

			Vec3 v_rel = vel_a - vel_b;

			float v_rel_dot_n = dot(v_rel, info.normalWorld);

			if (v_rel_dot_n > 0) // Bodies are already separating: do nothing
				continue;

			Vec3 x_a = body_a.getWorldToBody().transformVector(info.collisionPointWorld);
			Vec3 x_b = body_b.getWorldToBody().transformVector(info.collisionPointWorld);

			Vec3 bodyAImpulse = body_a.calcImpulse(info.collisionPointWorld, info.normalWorld);
			Vec3 bodyBImpulse = body_b.calcImpulse(info.collisionPointWorld, info.normalWorld);

			float impulse = -v_rel_dot_n / (1 / body_a.mass + 1 / body_b.mass + dot(bodyAImpulse + bodyBImpulse, info.normalWorld));

			body_a.linear_velocity = body_a.linear_velocity + (info.normalWorld * impulse) / body_a.mass;
			body_b.linear_velocity -= (impulse * info.normalWorld) / body_b.mass;

			body_a.angular_momentum += cross(x_a, impulse * info.normalWorld);
			body_b.angular_momentum -= cross(x_b, impulse * info.normalWorld);

			
			if (!body_a.canCollide) 
			{
				body_a.angular_momentum = (0, 0, 0);
				body_a.linear_velocity = (0, 0, 0);
			}

			if (!body_b.canCollide) 
			{
				body_b.angular_momentum = (0, 0, 0);
				body_b.linear_velocity = (0, 0, 0);
			}
			
		}

		if (body_a.center_of_mass.y > -1 && body_a.center_of_mass.y < 3 
			&& body_a.center_of_mass.x < 3 && body_a.center_of_mass.x > -3
			&& body_a.center_of_mass.z < 3 && body_a.center_of_mass.z > -3)
			continue;

		body_a.linear_velocity = -body_a.linear_velocity;

		

	
	}
}

void RigidBodySystemSimulator::handleCollisionWithMSS(RigidBody& body_a)
{
	if (body_a.canCollide) {
		MassSpringSystemSimulator mss = this->m_massSpringSystem;
		for (int j = 0; j < mss.mass_points.size(); j++)
		{
			Mat4 mp_to_world = mss.getMassPointToWorld(j);

			CollisionInfo info = checkCollisionSAT(body_a.getBodyToWorld(), mss.getMassPointToWorld(j));

			if (!info.isValid) // Bodies are not colliding: do nothing
				continue;

			//printf("Collision between body %d and mass point %d\n", i, j);
			Vec3 vel_a = body_a.getWorldVelocityAt(info.collisionPointWorld);
			Vec3 vel_b = mss.getVelocityOfMassPoint(j);

			Vec3 v_rel = vel_a - vel_b;

			float v_rel_dot_n = dot(v_rel, info.normalWorld);

			if (v_rel_dot_n > 0) // Bodies are already separating: do nothing
				continue;
			//W * p mit W = world space matrix, p = collision point
			Vec3 x_a = body_a.getWorldToBody().transformVector(info.collisionPointWorld);
			Vec3 x_b = mss.getMassPointToWorld(j).transformVector(info.collisionPointWorld);

			Vec3 bodyAImpulse = body_a.calcImpulse(info.collisionPointWorld, info.normalWorld);

			//mockup calcImpulse for mass points	
			Vec3 body_location = mp_to_world.transformVector(info.collisionPointWorld);
			Vec3 bodyBImpulse = cross(cross(body_location, info.normalWorld), body_location);
			float mass = mss.m_fMass;
			float impulse = -v_rel_dot_n / (1 / body_a.mass + 1 / mass + dot(bodyAImpulse + bodyBImpulse, info.normalWorld));

			//normalize impulse somehow
			impulse /= mss.mass_points.size();

			body_a.linear_velocity = body_a.linear_velocity + (info.normalWorld * impulse) / body_a.mass;
			mss.setVelocityOfMassPoint(j, (impulse * info.normalWorld) / mass);

			body_a.angular_momentum += cross(x_a, impulse * info.normalWorld);
		}
	}
	
}

void RigidBodySystemSimulator::onSpace()
{
	//use jedi powers
	//apply force to all bodies in the direction of the camera
	Vec3 eyePoint = Vec3(DUC->g_camera.GetEyePt());
	for (int i = 0;i < bodies.size();i++)
	{
		RigidBody& body = bodies[i];
		Vec3 center = body.center_of_mass;
		Vec3 force = (center - eyePoint);
		double size = sqrt(force.x * force.x + force.y * force.y + force.z * force.z);
		double current_force_size = sqrt(body.linear_velocity.x * body.linear_velocity.x + body.linear_velocity.y * body.linear_velocity.y + body.linear_velocity.z * body.linear_velocity.z);
		force /= size;
		//force *= max(1.0, current_force_size);
		Vec3 loc = center + force * 0.5;
		applyForceOnBody(i, loc, force);
	}
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
	onMouse(x, y);

	for (int i = 0; i < bodies.size(); i++)
	{
		RigidBody& body = bodies[i];
		setVelocityOf(i, Vec3());
	}
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
	this->m_massSpringSystem.onMouse(x, y);

	m_trackmouse.x = 0;
	m_trackmouse.y = 0;
}

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return bodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	return bodies[i].center_of_mass;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return bodies[i].linear_velocity;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return bodies[i].angular_velocity;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	
	bodies[i].add_force(force, loc);

}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass, bool canCollide)
{
	bodies.emplace_back(RigidBody::makeQuad(position, size, mass, canCollide));
}


void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	bodies[i].orientation = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	bodies[i].linear_velocity = velocity;
}
