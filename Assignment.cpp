#include "PhysicsGame1.h"
#include "PhysicsController.h"
#include "Sphere.h"
#include "PhysicsCamera.h"
#include "Box.h"
#include "Cylinder.h"
#include "Steerable3DController.h"
#include "Ground.h"
#include "Content.h"
#include <btBulletDynamicsCommon.h>
#include <gtc/quaternion.hpp>
#include <gtx/quaternion.hpp>
#include <gtx/euler_angles.hpp>
#include <gtx/norm.hpp>
#include "VectorDrawer.h"
#include "Utils.h"

#include "PhysicsFactory.h"
#include "Game.h" 
#include "Model.h"
#include "dirent.h"
#include "Capsule.h" 

#include "Assignment.h"

using namespace BGE;

Assignment::Assignment(void)
{
}

Assignment::~Assignment(void)
{
}


bool Assignment::Initialise()
{

	physicsFactory->CreateGroundPhysics();
	physicsFactory->CreateCameraPhysics();

	dynamicsWorld->setGravity(btVector3(0, -10, 0));


	shared_ptr<PhysicsController> crawler = CreateCrawler(glm::vec3(-10, 30, 0));


	if (!Game::Initialise()) {
		return false;
	}



	return true;
}

void BGE::Assignment::Update(float timeDelta)
{
	Game::Update(timeDelta);
}

void BGE::Assignment::Cleanup()
{
	Game::Cleanup();
}



shared_ptr<PhysicsController> Assignment::CreateCrawler(glm::vec3 position)
{
	//body
	glm::vec3 bodyPos = position;
	shared_ptr<PhysicsController> body = physicsFactory->CreateBox(bodyWidth, bodyLength, bodyDepth, bodyPos, glm::quat());

	//wheels
	glm::vec3 wheel1pos = glm::vec3(bodyPos.x + 5, bodyPos.y, bodyPos.z);
	shared_ptr<PhysicsController> wheel1 = physicsFactory->CreateCylinder(2, 1, wheel1pos, glm::angleAxis(0.0f, glm::vec3(1, 0, 0)));
	btHingeConstraint * wheel1hinge = new btHingeConstraint(*body->rigidBody, *wheel1->rigidBody, btVector3(-(bodyWidth/2), -(bodyLength/2), bodyDepth/2 + 2), btVector3(0, 0, 0), btVector3(0, 0, 1), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(wheel1hinge);

	glm::vec3 wheel2pos = glm::vec3(bodyPos.x + 5, bodyPos.y, bodyPos.z);
	shared_ptr<PhysicsController> wheel2 = physicsFactory->CreateCylinder(2, 1, wheel2pos, glm::angleAxis(0.0f, glm::vec3(1, 0, 0)));
	btHingeConstraint * wheel2hinge = new btHingeConstraint(*body->rigidBody, *wheel2->rigidBody, btVector3(-(bodyWidth / 2), -(bodyLength / 2), -(bodyDepth / 2 + 2)), btVector3(0, 0, 0), btVector3(0, 0, 1), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(wheel2hinge);

	glm::vec3 wheel3pos = glm::vec3(bodyPos.x + 5, bodyPos.y, bodyPos.z);
	shared_ptr<PhysicsController> wheel3 = physicsFactory->CreateCylinder(2, 1, wheel3pos, glm::angleAxis(0.0f, glm::vec3(1, 0, 0)));
	btHingeConstraint * wheel3hinge = new btHingeConstraint(*body->rigidBody, *wheel3->rigidBody, btVector3((bodyWidth / 2), -(bodyLength / 2), bodyDepth / 2 + 2), btVector3(0, 0, 0), btVector3(0, 0, 1), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(wheel3hinge);

	glm::vec3 wheel4pos = glm::vec3(bodyPos.x + 5, bodyPos.y, bodyPos.z);
	shared_ptr<PhysicsController> wheel4 = physicsFactory->CreateCylinder(2, 1, wheel4pos, glm::angleAxis(0.0f, glm::vec3(1, 0, 0)));
	btHingeConstraint * wheel4hinge = new btHingeConstraint(*body->rigidBody, *wheel4->rigidBody, btVector3((bodyWidth / 2), -(bodyLength / 2), -(bodyDepth / 2 + 2)), btVector3(0, 0, 0), btVector3(0, 0, 1), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(wheel4hinge);

	//arms

	shared_ptr<PhysicsController> leftArm = physicsFactory->CreateBox(armWidth, armLength, armDepth, bodyPos, glm::quat());
	shared_ptr<PhysicsController> leftHand = physicsFactory->CreateBox(armWidth/8, armLength, armDepth * 2, bodyPos, glm::quat());

	btTransform t1, t2;
	t1.setIdentity();
	t2.setIdentity();
	t1.setOrigin(btVector3(0, 0, 0));
	t2.setOrigin(btVector3((armWidth/2) + 1, 0, armDepth));
	btFixedConstraint * arm_hand = new btFixedConstraint(*leftArm->rigidBody, *leftHand->rigidBody, t1, t2);
	dynamicsWorld->addConstraint(arm_hand);

	btHingeConstraint * leftArmHinge = new btHingeConstraint(*body->rigidBody, *leftArm->rigidBody, btVector3(0, 0, (bodyDepth / 2 + 2)), btVector3(0, 0, 0), btVector3(0, 0, 1), btVector3(0, 1, 0), true);
	leftArmHinge->enableAngularMotor(true, 10, 20);
	dynamicsWorld->addConstraint(leftArmHinge);


	shared_ptr<PhysicsController> rightArm = physicsFactory->CreateBox(armWidth, armLength, armDepth, bodyPos, glm::quat());
	shared_ptr<PhysicsController> rightHand = physicsFactory->CreateBox(armWidth / 8, armLength, armDepth * 2, bodyPos, glm::quat());

	btTransform t3, t4;
	t3.setIdentity();
	t4.setIdentity();
	t3.setOrigin(btVector3(0, 0, 0));
	t4.setOrigin(btVector3((armWidth / 2) + 1, 0, armDepth));
	btFixedConstraint * right_arm_hand = new btFixedConstraint(*rightArm->rigidBody, *rightHand->rigidBody, t3, t4);
	dynamicsWorld->addConstraint(right_arm_hand);

	btHingeConstraint * rightArmHinge = new btHingeConstraint(*body->rigidBody, *rightArm->rigidBody, btVector3(0, 0, -(bodyDepth / 2 + 2)), btVector3(0, 0, 0), btVector3(0, 0, 1), btVector3(0, 1, 0), true);
	rightArmHinge->enableAngularMotor(true, 10, 20);
	dynamicsWorld->addConstraint(rightArmHinge);

	return body;
}