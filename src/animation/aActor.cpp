#include "aActor.h"

#pragma warning(disable : 4018)



/****************************************************************
*
*    	    Actor functions
*
****************************************************************/

AActor::AActor() 
{
	m_pInternalSkeleton = new ASkeleton();
	m_pSkeleton = m_pInternalSkeleton;

	m_BVHController = new BVHController();
	m_BVHController->setActor(this);

	m_IKController = new IKController();
	m_IKController->setActor(this);

	// code to update additional Actor data goes here
	resetGuide();

}

AActor::AActor(const AActor* actor)
{
	*this = *actor;
}

AActor& AActor::operator = (const AActor& actor)
{
	// Performs a deep copy
	if (&actor == this)
	{
		return *this;
	}
	m_pSkeleton = actor.m_pSkeleton;

	// code to update additional Actor data goes here


	return *this;
}

AActor::~AActor()
{
	 delete m_IKController;
	 delete m_BVHController;
	 delete m_pInternalSkeleton;

}

void AActor::clear()
{
	// looks like it is clearing more times than the number of actors.  as a result, m_pSkeleton is not defined for last case.
	m_pSkeleton->clear();  

	// code to update additional Actor data goes here
}

void AActor::update()
{
	if (!m_pSkeleton->getRootNode() )
		 return; // Nothing loaded
	else m_pSkeleton->update();

	// code to update additional Actor data goes here

}

ASkeleton* AActor::getSkeleton()
{
	return m_pSkeleton;
}

void AActor::setSkeleton(ASkeleton* pExternalSkeleton)
{
	m_pSkeleton = pExternalSkeleton;
}

void AActor::resetSkeleton()
{
	m_pSkeleton = m_pInternalSkeleton;
}

BVHController* AActor::getBVHController()
{
	return m_BVHController;
}

IKController* AActor::getIKController()
{
	return m_IKController;
}

void AActor::updateGuideJoint(vec3 guideTargetPos)
{
	if (!m_pSkeleton->getRootNode()) { return; }

	// TODO: 
	// 1.	Set the global position of the guide joint to the global position of the root joint
	// 2.	Set the y component of the guide position to 0
	AJoint* root = m_pSkeleton->getRootNode();
	vec3 pos = m_Guide.getGlobalRotation() * root->getGlobalTranslation() + m_Guide.getGlobalTranslation();
	pos[1] = 0;
	m_Guide.setGlobalTranslation(pos);

	// 3.	Set the global rotation of the guide joint towards the guideTarget
	guideTargetPos[1] = 0;
	vec3 forward = (guideTargetPos - m_Guide.getGlobalTranslation()).Normalize();
	vec3 up = vec3(0, 1, 0);
	mat3 rot(up.Cross(forward).Normalize(), up, forward);
	m_Guide.setGlobalRotation(rot.Transpose());
	m_pSkeleton->update();

}

void AActor::solveFootIK(float leftHeight, float rightHeight, bool rotateLeft, bool rotateRight, vec3 leftNormal, vec3 rightNormal)
{
	if (!m_pSkeleton->getRootNode()) { return; }
	AJoint* leftFoot = m_pSkeleton->getJointByID(m_IKController->mLfootID);
	AJoint* rightFoot = m_pSkeleton->getJointByID(m_IKController->mRfootID);
	AJoint* root = m_pSkeleton->getRootNode();

	// TODO: 
	// The normal and the height given are in the world space

	// 1.	Update the local translation of the root based on the left height and the right height
	double height = leftHeight;
	if(rightHeight>leftHeight) height = rightHeight;
	vec3 rootPos = root->getLocalTranslation();
	root->setLocalTranslation(vec3(rootPos[0], rootPos[1]+height, rootPos[2]));
	m_pSkeleton->update();

	// 2.	Update the character with Limb-based IK 
	vec3 leftFootPos = leftFoot->getGlobalTranslation()+vec3(0, leftHeight, 0);
	ATarget targetLeft = ATarget();
	targetLeft.setGlobalTranslation(leftFootPos);
	m_IKController->IKSolver_Limb(m_IKController->mLfootID, targetLeft);

	vec3 rightFootPos = rightFoot->getGlobalTranslation()+vec3(0, rightHeight, 0);
	ATarget targetRight = ATarget();
	targetRight.setGlobalTranslation(rightFootPos);
	m_IKController->IKSolver_Limb(m_IKController->mRfootID, targetRight);
	
	// Rotate Foot
	if (rotateLeft)
	{
		// Update the local orientation of the left foot based on the left normal
		mat3 rot( leftNormal.Cross(leftFoot->getGlobalRotation()[2] ), leftNormal, leftFoot->getGlobalRotation()[2] );
		leftFoot->setGlobalRotation(rot);
	}
	if (rotateRight)
	{
		// Update the local orientation of the right foot based on the right normal
		mat3 rot( rightNormal.Cross(rightFoot->getGlobalRotation()[2] ), rightNormal, rightFoot->getGlobalRotation()[2] );
		rightFoot->setGlobalRotation(rot);

	}
	m_pSkeleton->update();

}
