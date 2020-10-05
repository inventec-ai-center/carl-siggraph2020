#pragma once


#include "anim/KinTree.h"

class cCarlKinTree: virtual public cKinTree
{
public:

	enum eJointDescExtra
	{
        eJointDescExtraEnableContact,
		eJointDescExtraContactThreshold,
		eJointDescExtraContactOffsetX,
		eJointDescExtraContactOffsetY,
		eJointDescExtraContactOffsetZ,
		eJointDescExtraMax
	};
	typedef Eigen::Matrix<double, 1, eJointDescExtraMax> tJointDescExtra;
    typedef Eigen::Matrix<double, 1, eJointDescMax + eJointDescExtraMax> tJointDescJointed;

    static bool Load(const Json::Value& root, Eigen::MatrixXd& out_joint_mat);

	static bool IsFootContact(const Eigen::MatrixXd& joint_mat, int joint_id);
	static double GetContactThreshold(const Eigen::MatrixXd& joint_mat, int joint_id);
	static tVector GetContactOffset(const Eigen::MatrixXd& joint_mat, int joint_id);
    static tJointDescExtra BuildJointDescExtra();

protected:

    static bool ParseJointExtra(const Json::Value& root, tJointDescExtra& out_joint_desc_extra);
};
