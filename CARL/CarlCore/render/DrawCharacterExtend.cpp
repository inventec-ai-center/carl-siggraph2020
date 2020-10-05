#include "DrawCharacterExtend.h" 
#include "DrawKinTreeExtend.h"

void cDrawCharacterExtend::DrawBones(const cCharacter& character, double link_width, const tVector& fill_col, const tVector& bone_col)
{
	glDisable(GL_DEPTH_TEST);
	const Eigen::MatrixXd& joint_mat = character.GetJointMat();
	const Eigen::VectorXd& pose = character.GetPose();
	cDrawKinTreeExtend::Draw(joint_mat, pose, link_width, fill_col, bone_col);
	glEnable(GL_DEPTH_TEST);
}

