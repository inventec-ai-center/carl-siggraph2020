#include "CarlKinTree.h"

const std::string gJointsKey = "Joints";
const std::string gJointDescExtraKeys[cCarlKinTree::eJointDescExtraMax] = 
{
	"EnableContact",
	"ContactThreshold",
	"ContactOffsetX",
	"ContactOffsetY",
	"ContactOffsetZ"
};


bool cCarlKinTree::Load(const Json::Value& root, Eigen::MatrixXd& out_joint_mat)
{
	bool succ = false;

	if (!root[gJointsKey].isNull())
	{
		Json::Value joints = root[gJointsKey];
		int num_joints = joints.size();

		out_joint_mat.resize(num_joints, eJointDescMax + eJointDescExtraMax);

		for (int j = 0; j < num_joints; ++j)
		{
			tJointDesc curr_joint_desc = tJointDesc::Zero();
            tJointDescExtra curr_joint_desc_extra = tJointDescExtra::Zero();

			Json::Value joint_json = joints.get(j, 0);
			succ = ParseJoint(joint_json, curr_joint_desc);
            succ &= ParseJointExtra(joint_json, curr_joint_desc_extra);
			if (succ)
			{
                tJointDescJointed curr_joint_desc_joined = tJointDescJointed::Zero();
                curr_joint_desc_joined << curr_joint_desc, curr_joint_desc_extra;
				out_joint_mat.row(j) = curr_joint_desc_joined;
			}
			else
			{
				printf("Failed to parse joint %i\n", j);
				return false;
			}
		}

		for (int j = 0; j < num_joints; ++j)
		{
			const auto& curr_desc = out_joint_mat.row(j);
			int parent_id = static_cast<int>(curr_desc(eJointDescParent));
			if (parent_id >= j)
			{
				printf("Parent id must be < child id, parent id: %i, child id: %i\n", parent_id, j);
				out_joint_mat.resize(0, 0);
				assert(false);

				return false;
			}

			out_joint_mat.row(j) = curr_desc;
		}

		PostProcessJointMat(out_joint_mat);
	}

	return succ;
}

bool cCarlKinTree::IsFootContact(const Eigen::MatrixXd& joint_mat, int joint_id)
{
	double foot_contact_val = joint_mat(joint_id, eJointDescMax + eJointDescExtraEnableContact);
	return foot_contact_val != 0;
}

double cCarlKinTree::GetContactThreshold(const Eigen::MatrixXd& joint_mat, int joint_id)
{
	double contact_threshold = joint_mat(joint_id, eJointDescMax + eJointDescExtraContactThreshold);
	return contact_threshold;
}

tVector cCarlKinTree::GetContactOffset(const Eigen::MatrixXd& joint_mat, int joint_id)
{
	return tVector(joint_mat(joint_id, eJointDescMax + eJointDescExtraContactOffsetX),
				   joint_mat(joint_id, eJointDescMax + eJointDescExtraContactOffsetY),
				   joint_mat(joint_id, eJointDescMax + eJointDescExtraContactOffsetZ), 1);
}

bool cCarlKinTree::ParseJointExtra(const Json::Value& root, tJointDescExtra& out_joint_desc_extra)
{
	out_joint_desc_extra = BuildJointDescExtra();
	for (int i = 0; i < eJointDescExtraMax; ++i)
	{
        const std::string& key = gJointDescExtraKeys[i];
        if (!root[key].isNull())
        {
            out_joint_desc_extra[i] = root[key].asDouble();
        }
	}
	return true;
}

cCarlKinTree::tJointDescExtra cCarlKinTree::BuildJointDescExtra()
{
	tJointDescExtra desc;
	desc(eJointDescExtraEnableContact) = 0;
	desc(eJointDescExtraContactThreshold) = 0;
	desc(eJointDescExtraContactOffsetX) = 0;
	desc(eJointDescExtraContactOffsetY) = 0;
	desc(eJointDescExtraContactOffsetZ) = 0;
	return desc;
}
