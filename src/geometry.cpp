#include "geometry.h"

namespace aito
{

AnimatedTransform::AnimatedTransform(const Transform* start_transform, const Transform* end_transform, Float start_time, Float end_time)
	: start_transform_(start_transform), end_transform_(end_transform), start_time_(start_time), end_time_(end_time), animated_(*start_transform_ != *end_transform_)
{
	decompose(start_transform_->m_, T_[0], R_[0], S_[0]);
	decompose(end_transform_->m_, T_[1], R_[1], S_[1]);

	// Flip r if needed
	if (glm::dot(R_[0], R_[1]) < 0)
		R_[1] = -R_[1];

	has_rotation_ = glm::dot(R_[0], R_[1]) < 0.9995f;

    // I would like to give a personal thank you to the authors of pbrt, because I am not rewriting this. Copy paste it is.
    if (has_rotation_)
    {
        Float cosTheta = glm::dot(R_[0], R_[1]);
        Float theta = std::acos(std::clamp<Float>(cosTheta, -1, 1));
        Quaternion qperp = glm::normalize(R_[1] - R_[0] * cosTheta);

        Float t0x = T_[0].x;
        Float t0y = T_[0].y;
        Float t0z = T_[0].z;
        Float t1x = T_[1].x;
        Float t1y = T_[1].y;
        Float t1z = T_[1].z;
        Float q0x = R_[0].x;
        Float q0y = R_[0].y;
        Float q0z = R_[0].z;
        Float q0w = R_[0].w;
        Float qperpx = qperp.x;
        Float qperpy = qperp.y;
        Float qperpz = qperp.z;
        Float qperpw = qperp.w;
        Float s000 = S_[0][0][0];
        Float s001 = S_[0][0][1];
        Float s002 = S_[0][0][2];
        Float s010 = S_[0][1][0];
        Float s011 = S_[0][1][1];
        Float s012 = S_[0][1][2];
        Float s020 = S_[0][2][0];
        Float s021 = S_[0][2][1];
        Float s022 = S_[0][2][2];
        Float s100 = S_[1][0][0];
        Float s101 = S_[1][0][1];
        Float s102 = S_[1][0][2];
        Float s110 = S_[1][1][0];
        Float s111 = S_[1][1][1];
        Float s112 = S_[1][1][2];
        Float s120 = S_[1][2][0];
        Float s121 = S_[1][2][1];
        Float s122 = S_[1][2][2];

        c1_[0] = DerivativeTerm(
            -t0x + t1x,
            (-1 + q0y * q0y + q0z * q0z + qperpy * qperpy + qperpz * qperpz) *
            s000 +
            q0w * q0z * s010 - qperpx * qperpy * s010 +
            qperpw * qperpz * s010 - q0w * q0y * s020 -
            qperpw * qperpy * s020 - qperpx * qperpz * s020 + s100 -
            q0y * q0y * s100 - q0z * q0z * s100 - qperpy * qperpy * s100 -
            qperpz * qperpz * s100 - q0w * q0z * s110 +
            qperpx * qperpy * s110 - qperpw * qperpz * s110 +
            q0w * q0y * s120 + qperpw * qperpy * s120 +
            qperpx * qperpz * s120 +
            q0x * (-(q0y * s010) - q0z * s020 + q0y * s110 + q0z * s120),
            (-1 + q0y * q0y + q0z * q0z + qperpy * qperpy + qperpz * qperpz) *
            s001 +
            q0w * q0z * s011 - qperpx * qperpy * s011 +
            qperpw * qperpz * s011 - q0w * q0y * s021 -
            qperpw * qperpy * s021 - qperpx * qperpz * s021 + s101 -
            q0y * q0y * s101 - q0z * q0z * s101 - qperpy * qperpy * s101 -
            qperpz * qperpz * s101 - q0w * q0z * s111 +
            qperpx * qperpy * s111 - qperpw * qperpz * s111 +
            q0w * q0y * s121 + qperpw * qperpy * s121 +
            qperpx * qperpz * s121 +
            q0x * (-(q0y * s011) - q0z * s021 + q0y * s111 + q0z * s121),
            (-1 + q0y * q0y + q0z * q0z + qperpy * qperpy + qperpz * qperpz) *
            s002 +
            q0w * q0z * s012 - qperpx * qperpy * s012 +
            qperpw * qperpz * s012 - q0w * q0y * s022 -
            qperpw * qperpy * s022 - qperpx * qperpz * s022 + s102 -
            q0y * q0y * s102 - q0z * q0z * s102 - qperpy * qperpy * s102 -
            qperpz * qperpz * s102 - q0w * q0z * s112 +
            qperpx * qperpy * s112 - qperpw * qperpz * s112 +
            q0w * q0y * s122 + qperpw * qperpy * s122 +
            qperpx * qperpz * s122 +
            q0x * (-(q0y * s012) - q0z * s022 + q0y * s112 + q0z * s122));

        c2_[0] = DerivativeTerm(
            0.,
            -(qperpy * qperpy * s000) - qperpz * qperpz * s000 +
            qperpx * qperpy * s010 - qperpw * qperpz * s010 +
            qperpw * qperpy * s020 + qperpx * qperpz * s020 +
            q0y * q0y * (s000 - s100) + q0z * q0z * (s000 - s100) +
            qperpy * qperpy * s100 + qperpz * qperpz * s100 -
            qperpx * qperpy * s110 + qperpw * qperpz * s110 -
            qperpw * qperpy * s120 - qperpx * qperpz * s120 +
            2 * q0x * qperpy * s010 * theta -
            2 * q0w * qperpz * s010 * theta +
            2 * q0w * qperpy * s020 * theta +
            2 * q0x * qperpz * s020 * theta +
            q0y *
            (q0x * (-s010 + s110) + q0w * (-s020 + s120) +
            2 * (-2 * qperpy * s000 + qperpx * s010 + qperpw * s020) *
            theta) +
            q0z * (q0w * (s010 - s110) + q0x * (-s020 + s120) -
            2 * (2 * qperpz * s000 + qperpw * s010 - qperpx * s020) *
            theta),
            -(qperpy * qperpy * s001) - qperpz * qperpz * s001 +
            qperpx * qperpy * s011 - qperpw * qperpz * s011 +
            qperpw * qperpy * s021 + qperpx * qperpz * s021 +
            q0y * q0y * (s001 - s101) + q0z * q0z * (s001 - s101) +
            qperpy * qperpy * s101 + qperpz * qperpz * s101 -
            qperpx * qperpy * s111 + qperpw * qperpz * s111 -
            qperpw * qperpy * s121 - qperpx * qperpz * s121 +
            2 * q0x * qperpy * s011 * theta -
            2 * q0w * qperpz * s011 * theta +
            2 * q0w * qperpy * s021 * theta +
            2 * q0x * qperpz * s021 * theta +
            q0y *
            (q0x * (-s011 + s111) + q0w * (-s021 + s121) +
            2 * (-2 * qperpy * s001 + qperpx * s011 + qperpw * s021) *
            theta) +
            q0z * (q0w * (s011 - s111) + q0x * (-s021 + s121) -
            2 * (2 * qperpz * s001 + qperpw * s011 - qperpx * s021) *
            theta),
            -(qperpy * qperpy * s002) - qperpz * qperpz * s002 +
            qperpx * qperpy * s012 - qperpw * qperpz * s012 +
            qperpw * qperpy * s022 + qperpx * qperpz * s022 +
            q0y * q0y * (s002 - s102) + q0z * q0z * (s002 - s102) +
            qperpy * qperpy * s102 + qperpz * qperpz * s102 -
            qperpx * qperpy * s112 + qperpw * qperpz * s112 -
            qperpw * qperpy * s122 - qperpx * qperpz * s122 +
            2 * q0x * qperpy * s012 * theta -
            2 * q0w * qperpz * s012 * theta +
            2 * q0w * qperpy * s022 * theta +
            2 * q0x * qperpz * s022 * theta +
            q0y *
            (q0x * (-s012 + s112) + q0w * (-s022 + s122) +
            2 * (-2 * qperpy * s002 + qperpx * s012 + qperpw * s022) *
            theta) +
            q0z * (q0w * (s012 - s112) + q0x * (-s022 + s122) -
            2 * (2 * qperpz * s002 + qperpw * s012 - qperpx * s022) *
            theta));

        c3_[0] = DerivativeTerm(
            0.,
            -2 * (q0x * qperpy * s010 - q0w * qperpz * s010 +
            q0w * qperpy * s020 + q0x * qperpz * s020 -
            q0x * qperpy * s110 + q0w * qperpz * s110 -
            q0w * qperpy * s120 - q0x * qperpz * s120 +
            q0y * (-2 * qperpy * s000 + qperpx * s010 + qperpw * s020 +
            2 * qperpy * s100 - qperpx * s110 - qperpw * s120) +
            q0z * (-2 * qperpz * s000 - qperpw * s010 + qperpx * s020 +
            2 * qperpz * s100 + qperpw * s110 - qperpx * s120)) *
            theta,
            -2 * (q0x * qperpy * s011 - q0w * qperpz * s011 +
            q0w * qperpy * s021 + q0x * qperpz * s021 -
            q0x * qperpy * s111 + q0w * qperpz * s111 -
            q0w * qperpy * s121 - q0x * qperpz * s121 +
            q0y * (-2 * qperpy * s001 + qperpx * s011 + qperpw * s021 +
            2 * qperpy * s101 - qperpx * s111 - qperpw * s121) +
            q0z * (-2 * qperpz * s001 - qperpw * s011 + qperpx * s021 +
            2 * qperpz * s101 + qperpw * s111 - qperpx * s121)) *
            theta,
            -2 * (q0x * qperpy * s012 - q0w * qperpz * s012 +
            q0w * qperpy * s022 + q0x * qperpz * s022 -
            q0x * qperpy * s112 + q0w * qperpz * s112 -
            q0w * qperpy * s122 - q0x * qperpz * s122 +
            q0y * (-2 * qperpy * s002 + qperpx * s012 + qperpw * s022 +
            2 * qperpy * s102 - qperpx * s112 - qperpw * s122) +
            q0z * (-2 * qperpz * s002 - qperpw * s012 + qperpx * s022 +
            2 * qperpz * s102 + qperpw * s112 - qperpx * s122)) *
            theta);

        c4_[0] = DerivativeTerm(
            0.,
            -(q0x * qperpy * s010) + q0w * qperpz * s010 - q0w * qperpy * s020 -
            q0x * qperpz * s020 + q0x * qperpy * s110 -
            q0w * qperpz * s110 + q0w * qperpy * s120 +
            q0x * qperpz * s120 + 2 * q0y * q0y * s000 * theta +
            2 * q0z * q0z * s000 * theta -
            2 * qperpy * qperpy * s000 * theta -
            2 * qperpz * qperpz * s000 * theta +
            2 * qperpx * qperpy * s010 * theta -
            2 * qperpw * qperpz * s010 * theta +
            2 * qperpw * qperpy * s020 * theta +
            2 * qperpx * qperpz * s020 * theta +
            q0y * (-(qperpx * s010) - qperpw * s020 +
            2 * qperpy * (s000 - s100) + qperpx * s110 +
            qperpw * s120 - 2 * q0x * s010 * theta -
            2 * q0w * s020 * theta) +
            q0z * (2 * qperpz * s000 + qperpw * s010 - qperpx * s020 -
            2 * qperpz * s100 - qperpw * s110 + qperpx * s120 +
            2 * q0w * s010 * theta - 2 * q0x * s020 * theta),
            -(q0x * qperpy * s011) + q0w * qperpz * s011 - q0w * qperpy * s021 -
            q0x * qperpz * s021 + q0x * qperpy * s111 -
            q0w * qperpz * s111 + q0w * qperpy * s121 +
            q0x * qperpz * s121 + 2 * q0y * q0y * s001 * theta +
            2 * q0z * q0z * s001 * theta -
            2 * qperpy * qperpy * s001 * theta -
            2 * qperpz * qperpz * s001 * theta +
            2 * qperpx * qperpy * s011 * theta -
            2 * qperpw * qperpz * s011 * theta +
            2 * qperpw * qperpy * s021 * theta +
            2 * qperpx * qperpz * s021 * theta +
            q0y * (-(qperpx * s011) - qperpw * s021 +
            2 * qperpy * (s001 - s101) + qperpx * s111 +
            qperpw * s121 - 2 * q0x * s011 * theta -
            2 * q0w * s021 * theta) +
            q0z * (2 * qperpz * s001 + qperpw * s011 - qperpx * s021 -
            2 * qperpz * s101 - qperpw * s111 + qperpx * s121 +
            2 * q0w * s011 * theta - 2 * q0x * s021 * theta),
            -(q0x * qperpy * s012) + q0w * qperpz * s012 - q0w * qperpy * s022 -
            q0x * qperpz * s022 + q0x * qperpy * s112 -
            q0w * qperpz * s112 + q0w * qperpy * s122 +
            q0x * qperpz * s122 + 2 * q0y * q0y * s002 * theta +
            2 * q0z * q0z * s002 * theta -
            2 * qperpy * qperpy * s002 * theta -
            2 * qperpz * qperpz * s002 * theta +
            2 * qperpx * qperpy * s012 * theta -
            2 * qperpw * qperpz * s012 * theta +
            2 * qperpw * qperpy * s022 * theta +
            2 * qperpx * qperpz * s022 * theta +
            q0y * (-(qperpx * s012) - qperpw * s022 +
            2 * qperpy * (s002 - s102) + qperpx * s112 +
            qperpw * s122 - 2 * q0x * s012 * theta -
            2 * q0w * s022 * theta) +
            q0z * (2 * qperpz * s002 + qperpw * s012 - qperpx * s022 -
            2 * qperpz * s102 - qperpw * s112 + qperpx * s122 +
            2 * q0w * s012 * theta - 2 * q0x * s022 * theta));

        c5_[0] = DerivativeTerm(
            0.,
            2 * (qperpy * qperpy * s000 + qperpz * qperpz * s000 -
            qperpx * qperpy * s010 + qperpw * qperpz * s010 -
            qperpw * qperpy * s020 - qperpx * qperpz * s020 -
            qperpy * qperpy * s100 - qperpz * qperpz * s100 +
            q0y * q0y * (-s000 + s100) + q0z * q0z * (-s000 + s100) +
            qperpx * qperpy * s110 - qperpw * qperpz * s110 +
            q0y * (q0x * (s010 - s110) + q0w * (s020 - s120)) +
            qperpw * qperpy * s120 + qperpx * qperpz * s120 +
            q0z * (-(q0w * s010) + q0x * s020 + q0w * s110 - q0x * s120)) *
            theta,
            2 * (qperpy * qperpy * s001 + qperpz * qperpz * s001 -
            qperpx * qperpy * s011 + qperpw * qperpz * s011 -
            qperpw * qperpy * s021 - qperpx * qperpz * s021 -
            qperpy * qperpy * s101 - qperpz * qperpz * s101 +
            q0y * q0y * (-s001 + s101) + q0z * q0z * (-s001 + s101) +
            qperpx * qperpy * s111 - qperpw * qperpz * s111 +
            q0y * (q0x * (s011 - s111) + q0w * (s021 - s121)) +
            qperpw * qperpy * s121 + qperpx * qperpz * s121 +
            q0z * (-(q0w * s011) + q0x * s021 + q0w * s111 - q0x * s121)) *
            theta,
            2 * (qperpy * qperpy * s002 + qperpz * qperpz * s002 -
            qperpx * qperpy * s012 + qperpw * qperpz * s012 -
            qperpw * qperpy * s022 - qperpx * qperpz * s022 -
            qperpy * qperpy * s102 - qperpz * qperpz * s102 +
            q0y * q0y * (-s002 + s102) + q0z * q0z * (-s002 + s102) +
            qperpx * qperpy * s112 - qperpw * qperpz * s112 +
            q0y * (q0x * (s012 - s112) + q0w * (s022 - s122)) +
            qperpw * qperpy * s122 + qperpx * qperpz * s122 +
            q0z * (-(q0w * s012) + q0x * s022 + q0w * s112 - q0x * s122)) *
            theta);

        c1_[1] = DerivativeTerm(
            -t0y + t1y,
            -(qperpx * qperpy * s000) - qperpw * qperpz * s000 - s010 +
            q0z * q0z * s010 + qperpx * qperpx * s010 +
            qperpz * qperpz * s010 - q0y * q0z * s020 +
            qperpw * qperpx * s020 - qperpy * qperpz * s020 +
            qperpx * qperpy * s100 + qperpw * qperpz * s100 +
            q0w * q0z * (-s000 + s100) + q0x * q0x * (s010 - s110) + s110 -
            q0z * q0z * s110 - qperpx * qperpx * s110 -
            qperpz * qperpz * s110 +
            q0x * (q0y * (-s000 + s100) + q0w * (s020 - s120)) +
            q0y * q0z * s120 - qperpw * qperpx * s120 +
            qperpy * qperpz * s120,
            -(qperpx * qperpy * s001) - qperpw * qperpz * s001 - s011 +
            q0z * q0z * s011 + qperpx * qperpx * s011 +
            qperpz * qperpz * s011 - q0y * q0z * s021 +
            qperpw * qperpx * s021 - qperpy * qperpz * s021 +
            qperpx * qperpy * s101 + qperpw * qperpz * s101 +
            q0w * q0z * (-s001 + s101) + q0x * q0x * (s011 - s111) + s111 -
            q0z * q0z * s111 - qperpx * qperpx * s111 -
            qperpz * qperpz * s111 +
            q0x * (q0y * (-s001 + s101) + q0w * (s021 - s121)) +
            q0y * q0z * s121 - qperpw * qperpx * s121 +
            qperpy * qperpz * s121,
            -(qperpx * qperpy * s002) - qperpw * qperpz * s002 - s012 +
            q0z * q0z * s012 + qperpx * qperpx * s012 +
            qperpz * qperpz * s012 - q0y * q0z * s022 +
            qperpw * qperpx * s022 - qperpy * qperpz * s022 +
            qperpx * qperpy * s102 + qperpw * qperpz * s102 +
            q0w * q0z * (-s002 + s102) + q0x * q0x * (s012 - s112) + s112 -
            q0z * q0z * s112 - qperpx * qperpx * s112 -
            qperpz * qperpz * s112 +
            q0x * (q0y * (-s002 + s102) + q0w * (s022 - s122)) +
            q0y * q0z * s122 - qperpw * qperpx * s122 +
            qperpy * qperpz * s122);

        c2_[1] = DerivativeTerm(
            0.,
            qperpx * qperpy * s000 + qperpw * qperpz * s000 + q0z * q0z * s010 -
            qperpx * qperpx * s010 - qperpz * qperpz * s010 -
            q0y * q0z * s020 - qperpw * qperpx * s020 +
            qperpy * qperpz * s020 - qperpx * qperpy * s100 -
            qperpw * qperpz * s100 + q0x * q0x * (s010 - s110) -
            q0z * q0z * s110 + qperpx * qperpx * s110 +
            qperpz * qperpz * s110 + q0y * q0z * s120 +
            qperpw * qperpx * s120 - qperpy * qperpz * s120 +
            2 * q0z * qperpw * s000 * theta +
            2 * q0y * qperpx * s000 * theta -
            4 * q0z * qperpz * s010 * theta +
            2 * q0z * qperpy * s020 * theta +
            2 * q0y * qperpz * s020 * theta +
            q0x * (q0w * s020 + q0y * (-s000 + s100) - q0w * s120 +
            2 * qperpy * s000 * theta - 4 * qperpx * s010 * theta -
            2 * qperpw * s020 * theta) +
            q0w * (-(q0z * s000) + q0z * s100 + 2 * qperpz * s000 * theta -
            2 * qperpx * s020 * theta),
            qperpx * qperpy * s001 + qperpw * qperpz * s001 + q0z * q0z * s011 -
            qperpx * qperpx * s011 - qperpz * qperpz * s011 -
            q0y * q0z * s021 - qperpw * qperpx * s021 +
            qperpy * qperpz * s021 - qperpx * qperpy * s101 -
            qperpw * qperpz * s101 + q0x * q0x * (s011 - s111) -
            q0z * q0z * s111 + qperpx * qperpx * s111 +
            qperpz * qperpz * s111 + q0y * q0z * s121 +
            qperpw * qperpx * s121 - qperpy * qperpz * s121 +
            2 * q0z * qperpw * s001 * theta +
            2 * q0y * qperpx * s001 * theta -
            4 * q0z * qperpz * s011 * theta +
            2 * q0z * qperpy * s021 * theta +
            2 * q0y * qperpz * s021 * theta +
            q0x * (q0w * s021 + q0y * (-s001 + s101) - q0w * s121 +
            2 * qperpy * s001 * theta - 4 * qperpx * s011 * theta -
            2 * qperpw * s021 * theta) +
            q0w * (-(q0z * s001) + q0z * s101 + 2 * qperpz * s001 * theta -
            2 * qperpx * s021 * theta),
            qperpx * qperpy * s002 + qperpw * qperpz * s002 + q0z * q0z * s012 -
            qperpx * qperpx * s012 - qperpz * qperpz * s012 -
            q0y * q0z * s022 - qperpw * qperpx * s022 +
            qperpy * qperpz * s022 - qperpx * qperpy * s102 -
            qperpw * qperpz * s102 + q0x * q0x * (s012 - s112) -
            q0z * q0z * s112 + qperpx * qperpx * s112 +
            qperpz * qperpz * s112 + q0y * q0z * s122 +
            qperpw * qperpx * s122 - qperpy * qperpz * s122 +
            2 * q0z * qperpw * s002 * theta +
            2 * q0y * qperpx * s002 * theta -
            4 * q0z * qperpz * s012 * theta +
            2 * q0z * qperpy * s022 * theta +
            2 * q0y * qperpz * s022 * theta +
            q0x * (q0w * s022 + q0y * (-s002 + s102) - q0w * s122 +
            2 * qperpy * s002 * theta - 4 * qperpx * s012 * theta -
            2 * qperpw * s022 * theta) +
            q0w * (-(q0z * s002) + q0z * s102 + 2 * qperpz * s002 * theta -
            2 * qperpx * s022 * theta));

        c3_[1] = DerivativeTerm(
            0., 2 * (-(q0x * qperpy * s000) - q0w * qperpz * s000 +
            2 * q0x * qperpx * s010 + q0x * qperpw * s020 +
            q0w * qperpx * s020 + q0x * qperpy * s100 +
            q0w * qperpz * s100 - 2 * q0x * qperpx * s110 -
            q0x * qperpw * s120 - q0w * qperpx * s120 +
            q0z * (2 * qperpz * s010 - qperpy * s020 +
            qperpw * (-s000 + s100) - 2 * qperpz * s110 +
            qperpy * s120) +
            q0y * (-(qperpx * s000) - qperpz * s020 + qperpx * s100 +
            qperpz * s120)) *
            theta,
            2 * (-(q0x * qperpy * s001) - q0w * qperpz * s001 +
            2 * q0x * qperpx * s011 + q0x * qperpw * s021 +
            q0w * qperpx * s021 + q0x * qperpy * s101 +
            q0w * qperpz * s101 - 2 * q0x * qperpx * s111 -
            q0x * qperpw * s121 - q0w * qperpx * s121 +
            q0z * (2 * qperpz * s011 - qperpy * s021 +
            qperpw * (-s001 + s101) - 2 * qperpz * s111 +
            qperpy * s121) +
            q0y * (-(qperpx * s001) - qperpz * s021 + qperpx * s101 +
            qperpz * s121)) *
            theta,
            2 * (-(q0x * qperpy * s002) - q0w * qperpz * s002 +
            2 * q0x * qperpx * s012 + q0x * qperpw * s022 +
            q0w * qperpx * s022 + q0x * qperpy * s102 +
            q0w * qperpz * s102 - 2 * q0x * qperpx * s112 -
            q0x * qperpw * s122 - q0w * qperpx * s122 +
            q0z * (2 * qperpz * s012 - qperpy * s022 +
            qperpw * (-s002 + s102) - 2 * qperpz * s112 +
            qperpy * s122) +
            q0y * (-(qperpx * s002) - qperpz * s022 + qperpx * s102 +
            qperpz * s122)) *
            theta);

        c4_[1] = DerivativeTerm(
            0.,
            -(q0x * qperpy * s000) - q0w * qperpz * s000 +
            2 * q0x * qperpx * s010 + q0x * qperpw * s020 +
            q0w * qperpx * s020 + q0x * qperpy * s100 +
            q0w * qperpz * s100 - 2 * q0x * qperpx * s110 -
            q0x * qperpw * s120 - q0w * qperpx * s120 +
            2 * qperpx * qperpy * s000 * theta +
            2 * qperpw * qperpz * s000 * theta +
            2 * q0x * q0x * s010 * theta + 2 * q0z * q0z * s010 * theta -
            2 * qperpx * qperpx * s010 * theta -
            2 * qperpz * qperpz * s010 * theta +
            2 * q0w * q0x * s020 * theta -
            2 * qperpw * qperpx * s020 * theta +
            2 * qperpy * qperpz * s020 * theta +
            q0y * (-(qperpx * s000) - qperpz * s020 + qperpx * s100 +
            qperpz * s120 - 2 * q0x * s000 * theta) +
            q0z * (2 * qperpz * s010 - qperpy * s020 +
            qperpw * (-s000 + s100) - 2 * qperpz * s110 +
            qperpy * s120 - 2 * q0w * s000 * theta -
            2 * q0y * s020 * theta),
            -(q0x * qperpy * s001) - q0w * qperpz * s001 +
            2 * q0x * qperpx * s011 + q0x * qperpw * s021 +
            q0w * qperpx * s021 + q0x * qperpy * s101 +
            q0w * qperpz * s101 - 2 * q0x * qperpx * s111 -
            q0x * qperpw * s121 - q0w * qperpx * s121 +
            2 * qperpx * qperpy * s001 * theta +
            2 * qperpw * qperpz * s001 * theta +
            2 * q0x * q0x * s011 * theta + 2 * q0z * q0z * s011 * theta -
            2 * qperpx * qperpx * s011 * theta -
            2 * qperpz * qperpz * s011 * theta +
            2 * q0w * q0x * s021 * theta -
            2 * qperpw * qperpx * s021 * theta +
            2 * qperpy * qperpz * s021 * theta +
            q0y * (-(qperpx * s001) - qperpz * s021 + qperpx * s101 +
            qperpz * s121 - 2 * q0x * s001 * theta) +
            q0z * (2 * qperpz * s011 - qperpy * s021 +
            qperpw * (-s001 + s101) - 2 * qperpz * s111 +
            qperpy * s121 - 2 * q0w * s001 * theta -
            2 * q0y * s021 * theta),
            -(q0x * qperpy * s002) - q0w * qperpz * s002 +
            2 * q0x * qperpx * s012 + q0x * qperpw * s022 +
            q0w * qperpx * s022 + q0x * qperpy * s102 +
            q0w * qperpz * s102 - 2 * q0x * qperpx * s112 -
            q0x * qperpw * s122 - q0w * qperpx * s122 +
            2 * qperpx * qperpy * s002 * theta +
            2 * qperpw * qperpz * s002 * theta +
            2 * q0x * q0x * s012 * theta + 2 * q0z * q0z * s012 * theta -
            2 * qperpx * qperpx * s012 * theta -
            2 * qperpz * qperpz * s012 * theta +
            2 * q0w * q0x * s022 * theta -
            2 * qperpw * qperpx * s022 * theta +
            2 * qperpy * qperpz * s022 * theta +
            q0y * (-(qperpx * s002) - qperpz * s022 + qperpx * s102 +
            qperpz * s122 - 2 * q0x * s002 * theta) +
            q0z * (2 * qperpz * s012 - qperpy * s022 +
            qperpw * (-s002 + s102) - 2 * qperpz * s112 +
            qperpy * s122 - 2 * q0w * s002 * theta -
            2 * q0y * s022 * theta));

        c5_[1] = DerivativeTerm(
            0., -2 * (qperpx * qperpy * s000 + qperpw * qperpz * s000 +
            q0z * q0z * s010 - qperpx * qperpx * s010 -
            qperpz * qperpz * s010 - q0y * q0z * s020 -
            qperpw * qperpx * s020 + qperpy * qperpz * s020 -
            qperpx * qperpy * s100 - qperpw * qperpz * s100 +
            q0w * q0z * (-s000 + s100) + q0x * q0x * (s010 - s110) -
            q0z * q0z * s110 + qperpx * qperpx * s110 +
            qperpz * qperpz * s110 +
            q0x * (q0y * (-s000 + s100) + q0w * (s020 - s120)) +
            q0y * q0z * s120 + qperpw * qperpx * s120 -
            qperpy * qperpz * s120) *
            theta,
            -2 * (qperpx * qperpy * s001 + qperpw * qperpz * s001 +
            q0z * q0z * s011 - qperpx * qperpx * s011 -
            qperpz * qperpz * s011 - q0y * q0z * s021 -
            qperpw * qperpx * s021 + qperpy * qperpz * s021 -
            qperpx * qperpy * s101 - qperpw * qperpz * s101 +
            q0w * q0z * (-s001 + s101) + q0x * q0x * (s011 - s111) -
            q0z * q0z * s111 + qperpx * qperpx * s111 +
            qperpz * qperpz * s111 +
            q0x * (q0y * (-s001 + s101) + q0w * (s021 - s121)) +
            q0y * q0z * s121 + qperpw * qperpx * s121 -
            qperpy * qperpz * s121) *
            theta,
            -2 * (qperpx * qperpy * s002 + qperpw * qperpz * s002 +
            q0z * q0z * s012 - qperpx * qperpx * s012 -
            qperpz * qperpz * s012 - q0y * q0z * s022 -
            qperpw * qperpx * s022 + qperpy * qperpz * s022 -
            qperpx * qperpy * s102 - qperpw * qperpz * s102 +
            q0w * q0z * (-s002 + s102) + q0x * q0x * (s012 - s112) -
            q0z * q0z * s112 + qperpx * qperpx * s112 +
            qperpz * qperpz * s112 +
            q0x * (q0y * (-s002 + s102) + q0w * (s022 - s122)) +
            q0y * q0z * s122 + qperpw * qperpx * s122 -
            qperpy * qperpz * s122) *
            theta);

        c1_[2] = DerivativeTerm(
            -t0z + t1z, (qperpw * qperpy * s000 - qperpx * qperpz * s000 -
            q0y * q0z * s010 - qperpw * qperpx * s010 -
            qperpy * qperpz * s010 - s020 + q0y * q0y * s020 +
            qperpx * qperpx * s020 + qperpy * qperpy * s020 -
            qperpw * qperpy * s100 + qperpx * qperpz * s100 +
            q0x * q0z * (-s000 + s100) + q0y * q0z * s110 +
            qperpw * qperpx * s110 + qperpy * qperpz * s110 +
            q0w * (q0y * (s000 - s100) + q0x * (-s010 + s110)) +
            q0x * q0x * (s020 - s120) + s120 - q0y * q0y * s120 -
            qperpx * qperpx * s120 - qperpy * qperpy * s120),
            (qperpw * qperpy * s001 - qperpx * qperpz * s001 -
            q0y * q0z * s011 - qperpw * qperpx * s011 -
            qperpy * qperpz * s011 - s021 + q0y * q0y * s021 +
            qperpx * qperpx * s021 + qperpy * qperpy * s021 -
            qperpw * qperpy * s101 + qperpx * qperpz * s101 +
            q0x * q0z * (-s001 + s101) + q0y * q0z * s111 +
            qperpw * qperpx * s111 + qperpy * qperpz * s111 +
            q0w * (q0y * (s001 - s101) + q0x * (-s011 + s111)) +
            q0x * q0x * (s021 - s121) + s121 - q0y * q0y * s121 -
            qperpx * qperpx * s121 - qperpy * qperpy * s121),
            (qperpw * qperpy * s002 - qperpx * qperpz * s002 -
            q0y * q0z * s012 - qperpw * qperpx * s012 -
            qperpy * qperpz * s012 - s022 + q0y * q0y * s022 +
            qperpx * qperpx * s022 + qperpy * qperpy * s022 -
            qperpw * qperpy * s102 + qperpx * qperpz * s102 +
            q0x * q0z * (-s002 + s102) + q0y * q0z * s112 +
            qperpw * qperpx * s112 + qperpy * qperpz * s112 +
            q0w * (q0y * (s002 - s102) + q0x * (-s012 + s112)) +
            q0x * q0x * (s022 - s122) + s122 - q0y * q0y * s122 -
            qperpx * qperpx * s122 - qperpy * qperpy * s122));

        c2_[2] = DerivativeTerm(
            0.,
            (q0w * q0y * s000 - q0x * q0z * s000 - qperpw * qperpy * s000 +
            qperpx * qperpz * s000 - q0w * q0x * s010 - q0y * q0z * s010 +
            qperpw * qperpx * s010 + qperpy * qperpz * s010 +
            q0x * q0x * s020 + q0y * q0y * s020 - qperpx * qperpx * s020 -
            qperpy * qperpy * s020 - q0w * q0y * s100 + q0x * q0z * s100 +
            qperpw * qperpy * s100 - qperpx * qperpz * s100 +
            q0w * q0x * s110 + q0y * q0z * s110 - qperpw * qperpx * s110 -
            qperpy * qperpz * s110 - q0x * q0x * s120 - q0y * q0y * s120 +
            qperpx * qperpx * s120 + qperpy * qperpy * s120 -
            2 * q0y * qperpw * s000 * theta + 2 * q0z * qperpx * s000 * theta -
            2 * q0w * qperpy * s000 * theta + 2 * q0x * qperpz * s000 * theta +
            2 * q0x * qperpw * s010 * theta + 2 * q0w * qperpx * s010 * theta +
            2 * q0z * qperpy * s010 * theta + 2 * q0y * qperpz * s010 * theta -
            4 * q0x * qperpx * s020 * theta - 4 * q0y * qperpy * s020 * theta),
            (q0w * q0y * s001 - q0x * q0z * s001 - qperpw * qperpy * s001 +
            qperpx * qperpz * s001 - q0w * q0x * s011 - q0y * q0z * s011 +
            qperpw * qperpx * s011 + qperpy * qperpz * s011 +
            q0x * q0x * s021 + q0y * q0y * s021 - qperpx * qperpx * s021 -
            qperpy * qperpy * s021 - q0w * q0y * s101 + q0x * q0z * s101 +
            qperpw * qperpy * s101 - qperpx * qperpz * s101 +
            q0w * q0x * s111 + q0y * q0z * s111 - qperpw * qperpx * s111 -
            qperpy * qperpz * s111 - q0x * q0x * s121 - q0y * q0y * s121 +
            qperpx * qperpx * s121 + qperpy * qperpy * s121 -
            2 * q0y * qperpw * s001 * theta + 2 * q0z * qperpx * s001 * theta -
            2 * q0w * qperpy * s001 * theta + 2 * q0x * qperpz * s001 * theta +
            2 * q0x * qperpw * s011 * theta + 2 * q0w * qperpx * s011 * theta +
            2 * q0z * qperpy * s011 * theta + 2 * q0y * qperpz * s011 * theta -
            4 * q0x * qperpx * s021 * theta - 4 * q0y * qperpy * s021 * theta),
            (q0w * q0y * s002 - q0x * q0z * s002 - qperpw * qperpy * s002 +
            qperpx * qperpz * s002 - q0w * q0x * s012 - q0y * q0z * s012 +
            qperpw * qperpx * s012 + qperpy * qperpz * s012 +
            q0x * q0x * s022 + q0y * q0y * s022 - qperpx * qperpx * s022 -
            qperpy * qperpy * s022 - q0w * q0y * s102 + q0x * q0z * s102 +
            qperpw * qperpy * s102 - qperpx * qperpz * s102 +
            q0w * q0x * s112 + q0y * q0z * s112 - qperpw * qperpx * s112 -
            qperpy * qperpz * s112 - q0x * q0x * s122 - q0y * q0y * s122 +
            qperpx * qperpx * s122 + qperpy * qperpy * s122 -
            2 * q0y * qperpw * s002 * theta + 2 * q0z * qperpx * s002 * theta -
            2 * q0w * qperpy * s002 * theta + 2 * q0x * qperpz * s002 * theta +
            2 * q0x * qperpw * s012 * theta + 2 * q0w * qperpx * s012 * theta +
            2 * q0z * qperpy * s012 * theta + 2 * q0y * qperpz * s012 * theta -
            4 * q0x * qperpx * s022 * theta -
            4 * q0y * qperpy * s022 * theta));

        c3_[2] = DerivativeTerm(
            0., -2 * (-(q0w * qperpy * s000) + q0x * qperpz * s000 +
            q0x * qperpw * s010 + q0w * qperpx * s010 -
            2 * q0x * qperpx * s020 + q0w * qperpy * s100 -
            q0x * qperpz * s100 - q0x * qperpw * s110 -
            q0w * qperpx * s110 +
            q0z * (qperpx * s000 + qperpy * s010 - qperpx * s100 -
            qperpy * s110) +
            2 * q0x * qperpx * s120 +
            q0y * (qperpz * s010 - 2 * qperpy * s020 +
            qperpw * (-s000 + s100) - qperpz * s110 +
            2 * qperpy * s120)) *
            theta,
            -2 * (-(q0w * qperpy * s001) + q0x * qperpz * s001 +
            q0x * qperpw * s011 + q0w * qperpx * s011 -
            2 * q0x * qperpx * s021 + q0w * qperpy * s101 -
            q0x * qperpz * s101 - q0x * qperpw * s111 -
            q0w * qperpx * s111 +
            q0z * (qperpx * s001 + qperpy * s011 - qperpx * s101 -
            qperpy * s111) +
            2 * q0x * qperpx * s121 +
            q0y * (qperpz * s011 - 2 * qperpy * s021 +
            qperpw * (-s001 + s101) - qperpz * s111 +
            2 * qperpy * s121)) *
            theta,
            -2 * (-(q0w * qperpy * s002) + q0x * qperpz * s002 +
            q0x * qperpw * s012 + q0w * qperpx * s012 -
            2 * q0x * qperpx * s022 + q0w * qperpy * s102 -
            q0x * qperpz * s102 - q0x * qperpw * s112 -
            q0w * qperpx * s112 +
            q0z * (qperpx * s002 + qperpy * s012 - qperpx * s102 -
            qperpy * s112) +
            2 * q0x * qperpx * s122 +
            q0y * (qperpz * s012 - 2 * qperpy * s022 +
            qperpw * (-s002 + s102) - qperpz * s112 +
            2 * qperpy * s122)) *
            theta);

        c4_[2] = DerivativeTerm(
            0.,
            q0w * qperpy * s000 - q0x * qperpz * s000 - q0x * qperpw * s010 -
            q0w * qperpx * s010 + 2 * q0x * qperpx * s020 -
            q0w * qperpy * s100 + q0x * qperpz * s100 +
            q0x * qperpw * s110 + q0w * qperpx * s110 -
            2 * q0x * qperpx * s120 - 2 * qperpw * qperpy * s000 * theta +
            2 * qperpx * qperpz * s000 * theta -
            2 * q0w * q0x * s010 * theta +
            2 * qperpw * qperpx * s010 * theta +
            2 * qperpy * qperpz * s010 * theta +
            2 * q0x * q0x * s020 * theta + 2 * q0y * q0y * s020 * theta -
            2 * qperpx * qperpx * s020 * theta -
            2 * qperpy * qperpy * s020 * theta +
            q0z * (-(qperpx * s000) - qperpy * s010 + qperpx * s100 +
            qperpy * s110 - 2 * q0x * s000 * theta) +
            q0y * (-(qperpz * s010) + 2 * qperpy * s020 +
            qperpw * (s000 - s100) + qperpz * s110 -
            2 * qperpy * s120 + 2 * q0w * s000 * theta -
            2 * q0z * s010 * theta),
            q0w * qperpy * s001 - q0x * qperpz * s001 - q0x * qperpw * s011 -
            q0w * qperpx * s011 + 2 * q0x * qperpx * s021 -
            q0w * qperpy * s101 + q0x * qperpz * s101 +
            q0x * qperpw * s111 + q0w * qperpx * s111 -
            2 * q0x * qperpx * s121 - 2 * qperpw * qperpy * s001 * theta +
            2 * qperpx * qperpz * s001 * theta -
            2 * q0w * q0x * s011 * theta +
            2 * qperpw * qperpx * s011 * theta +
            2 * qperpy * qperpz * s011 * theta +
            2 * q0x * q0x * s021 * theta + 2 * q0y * q0y * s021 * theta -
            2 * qperpx * qperpx * s021 * theta -
            2 * qperpy * qperpy * s021 * theta +
            q0z * (-(qperpx * s001) - qperpy * s011 + qperpx * s101 +
            qperpy * s111 - 2 * q0x * s001 * theta) +
            q0y * (-(qperpz * s011) + 2 * qperpy * s021 +
            qperpw * (s001 - s101) + qperpz * s111 -
            2 * qperpy * s121 + 2 * q0w * s001 * theta -
            2 * q0z * s011 * theta),
            q0w * qperpy * s002 - q0x * qperpz * s002 - q0x * qperpw * s012 -
            q0w * qperpx * s012 + 2 * q0x * qperpx * s022 -
            q0w * qperpy * s102 + q0x * qperpz * s102 +
            q0x * qperpw * s112 + q0w * qperpx * s112 -
            2 * q0x * qperpx * s122 - 2 * qperpw * qperpy * s002 * theta +
            2 * qperpx * qperpz * s002 * theta -
            2 * q0w * q0x * s012 * theta +
            2 * qperpw * qperpx * s012 * theta +
            2 * qperpy * qperpz * s012 * theta +
            2 * q0x * q0x * s022 * theta + 2 * q0y * q0y * s022 * theta -
            2 * qperpx * qperpx * s022 * theta -
            2 * qperpy * qperpy * s022 * theta +
            q0z * (-(qperpx * s002) - qperpy * s012 + qperpx * s102 +
            qperpy * s112 - 2 * q0x * s002 * theta) +
            q0y * (-(qperpz * s012) + 2 * qperpy * s022 +
            qperpw * (s002 - s102) + qperpz * s112 -
            2 * qperpy * s122 + 2 * q0w * s002 * theta -
            2 * q0z * s012 * theta));

        c5_[2] = DerivativeTerm(
            0., 2 * (qperpw * qperpy * s000 - qperpx * qperpz * s000 +
            q0y * q0z * s010 - qperpw * qperpx * s010 -
            qperpy * qperpz * s010 - q0y * q0y * s020 +
            qperpx * qperpx * s020 + qperpy * qperpy * s020 +
            q0x * q0z * (s000 - s100) - qperpw * qperpy * s100 +
            qperpx * qperpz * s100 +
            q0w * (q0y * (-s000 + s100) + q0x * (s010 - s110)) -
            q0y * q0z * s110 + qperpw * qperpx * s110 +
            qperpy * qperpz * s110 + q0y * q0y * s120 -
            qperpx * qperpx * s120 - qperpy * qperpy * s120 +
            q0x * q0x * (-s020 + s120)) *
            theta,
            2 * (qperpw * qperpy * s001 - qperpx * qperpz * s001 +
            q0y * q0z * s011 - qperpw * qperpx * s011 -
            qperpy * qperpz * s011 - q0y * q0y * s021 +
            qperpx * qperpx * s021 + qperpy * qperpy * s021 +
            q0x * q0z * (s001 - s101) - qperpw * qperpy * s101 +
            qperpx * qperpz * s101 +
            q0w * (q0y * (-s001 + s101) + q0x * (s011 - s111)) -
            q0y * q0z * s111 + qperpw * qperpx * s111 +
            qperpy * qperpz * s111 + q0y * q0y * s121 -
            qperpx * qperpx * s121 - qperpy * qperpy * s121 +
            q0x * q0x * (-s021 + s121)) *
            theta,
            2 * (qperpw * qperpy * s002 - qperpx * qperpz * s002 +
            q0y * q0z * s012 - qperpw * qperpx * s012 -
            qperpy * qperpz * s012 - q0y * q0y * s022 +
            qperpx * qperpx * s022 + qperpy * qperpy * s022 +
            q0x * q0z * (s002 - s102) - qperpw * qperpy * s102 +
            qperpx * qperpz * s102 +
            q0w * (q0y * (-s002 + s102) + q0x * (s012 - s112)) -
            q0y * q0z * s112 + qperpw * qperpx * s112 +
            qperpy * qperpz * s112 + q0y * q0y * s122 -
            qperpx * qperpx * s122 - qperpy * qperpy * s122 +
            q0x * q0x * (-s022 + s122)) *
            theta);
    };
}

Transform AnimatedTransform::interpolate(Float t) const
{
	if (!animated_ || t <= start_time_)
		return *start_transform_;
	if (t >= end_time_)
		return *end_transform_;

	Float dt =	(t - start_time_) / (end_time_ - t);

	// Compute translation
	Vec3f translation = lerp(dt, T_[0], T_[1]);

	// Compute the rotation
	Quaternion rotation = glm::slerp(R_[0], R_[1], dt);

	// Compute the scale
	Mat4 scale;
	for (size_t i = 0; i < 3; i++)
		for (size_t j = 0; j < 3; j++)
			scale[i][j] = lerp(dt, S_[0][i][j], S_[1][i][j]);

	auto r_matrix = glm::mat4_cast(rotation);
	return Transform::make_translation(translation) * Transform(r_matrix, glm::transpose(r_matrix)) * Transform(scale);
}

void AnimatedTransform::decompose(const Mat4& m, Vec3f& T_out, Quaternion& R_out, Mat4& S_out)
{
	// Extract translation
	T_out.x = m[0][3];
	T_out.y = m[1][3];
	T_out.z = m[2][3];

	// Copy matrix and remove translation
	Mat4 M = m;
	for (size_t i = 0; i < 3; i++)
		M[i][3] = 0;
	M[3][3] = 1.0f;

	// Solve for the rotational part
	Mat4 R = M;
	{
		Float norm;
		uint32_t count = 0;
		do
		{
			Mat4 R_next;
			Mat4 R_it = R.get_transpose().get_inverse();
			for (size_t i = 0; i < 4; i++)
				for (size_t j = 0; j < 4; j++)
					R_next[i][j] = 0.5f * (R[i][j] + R_it[i][j]);

			norm = 0;
			for (size_t i = 0; i < 3; i++)
			{
				Float n = 
					std::abs(R[i][0] - R_next[i][0]) +
					std::abs(R[i][1] - R_next[i][1]) +
					std::abs(R[i][2] - R_next[i][2]);
				norm = std::max(norm, n);
			}

			R = R_next;
		} while (count++ < 100 && norm < 0.001);
		R_out = glm::quat_cast(R.glm_matrix());
	}

	// Compute the scale
	S_out = R.get_inverse() * M;
}

class Interval
{
public:
    Float low, high;

public:
    constexpr Interval(Float v)
        : low(v), high(v)
    {}
    constexpr Interval(Float v0, Float v1)
        : low(std::min(v0, v1)), high(std::max(v0, v1))
    {}

    // Operators

    [[nodiscard]] constexpr Interval operator+(const Interval& i) const
    {
        return Interval(low + i.low, high + i.high);
    }
    [[nodiscard]] constexpr Interval operator-(const Interval& i) const
    {
        return Interval(low - i.high, high - i.low);
    }
    [[nodiscard]] constexpr Interval operator*(const Interval& i) const
    {
        return Interval(
            std::min(std::min(low * i.low, high * i.low), std::min(low * i.high, high * i.high)),
            std::max(std::max(low * i.low, high * i.low), std::max(low * i.high, high * i.high)));
    }

};

[[nodiscard]] inline Interval sin(const Interval& i)
{
    Float sinLow = std::sin(i.low), sinHigh = std::sin(i.high);
    if (sinLow > sinHigh) std::swap(sinLow, sinHigh);
    if (i.low < Pi / 2 && i.high > Pi / 2) sinHigh = 1.;
    if (i.low < (3.f / 2.f) * Pi && i.high >(3.f / 2.f) * Pi) sinLow = -1.;
    return Interval(sinLow, sinHigh);
}

[[nodiscard]] inline Interval cos(const Interval& i)
{
    Float cosLow = std::cos(i.low), cosHigh = std::cos(i.high);
    if (cosLow > cosHigh) std::swap(cosLow, cosHigh);
    if (i.low < Pi && i.high > Pi) cosLow = -1.;
    return Interval(cosLow, cosHigh);
}

void interval_find_zeros(Float c1, Float c2, Float c3, Float c4, Float c5, Float theta, Interval t_interval, std::array<Float, 4>& zeros_out, uint8_t& num_zeros_out, uint8_t depth = 8)
{
    // Eval motion derivative
    Interval range = Interval(c1) +
        (Interval(c2) + Interval(c2) * t_interval) *
        cos(Interval(2 * theta) * t_interval) +
        (Interval(c4) + Interval(c5) * t_interval) *
        sin(Interval(2 * theta) * t_interval);
    if (range.low > 0. || range.high < 0. || range.low == range.high)
        return;

    if (depth > 0U)
    {
        // Split interval and check both
        Float mid = (t_interval.low + t_interval.high) * 0.5f;
        interval_find_zeros(c1, c2, c3, c4, c5, theta,
                            Interval(t_interval.low, mid), zeros_out, num_zeros_out, depth - 1);
        interval_find_zeros(c1, c2, c3, c4, c5, theta,
                            Interval(mid, t_interval.high), zeros_out, num_zeros_out, depth - 1);
    }
    else
    {
        // Refine zero
        Float t_newton = (t_interval.low + t_interval.high) * 0.5f;

        for (uint8_t i = 0; i < 4; i++)
        {
            Float f_newton = c1 +
                (c2 + c3 * t_newton) * std::cos(2.f * theta * t_newton) +
                (c4 + c5 * t_newton) * std::sin(2.f * theta * t_newton);
            Float f_prime_newton = 
                (c3 + 2 * (c4 + c5 * t_newton) * theta) *
                std::cos(2.f * t_newton * theta) +
                (c5 - 2 * (c2 + c3 * t_newton) * theta) *
                std::sin(2.f * t_newton * theta);
            if (f_newton == 0 || f_prime_newton == 0)
                break;
            t_newton = t_newton - f_newton / f_prime_newton;
        }
        zeros_out[num_zeros_out] = t_newton;
        num_zeros_out++;
    }
};

Bounds3f AnimatedTransform::bound_point_motion(const Point3f& p) const
{
    Bounds3f bounds{(*start_transform_)(p), (*end_transform_)(p) };

    Float cosTheta = glm::dot(R_[0], R_[1]);
    Float theta = std::acos(std::clamp<Float>(cosTheta, -1, 1));
    for (size_t c = 0; c < 3; c++)
    {
        std::array<Float, 4> zeros; // There will at most be 4 maxima/minima.
        uint8_t num_zeros;
        interval_find_zeros(c1_[c].eval(p), c1_[c].eval(p), c1_[c].eval(p),
                            c1_[c].eval(p), c1_[c].eval(p), theta,
                            Interval(0, 1), zeros, num_zeros);
        for (uint8_t i = 0; i < num_zeros; i++)
        {
            auto T = interpolate(lerp(zeros[i], start_time_, end_time_));
            Point3f pz = T(p);
            bounds = bounds_union(bounds, pz);
        }
    }
    return bounds;
}

Bounds3f AnimatedTransform::motion_bounds(const Bounds3f& b) const
{
    if (!animated_)
        return (*start_transform_)(b);
    if (!has_rotation_)
        return bounds_union((*start_transform_)(b), (*end_transform_)(b));

    Bounds3f bounds;
    for (uint8_t i = 0; i < 8; i++)
        bounds = bounds_union(bounds, bound_point_motion(b.corner(i)));
    return bounds;
}

}
