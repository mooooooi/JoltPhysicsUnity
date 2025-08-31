﻿using System;
using Jolt;
using Unity.Collections;
using Unity.Mathematics;

namespace Jolt
{
    public partial struct HingeConstraint : IEquatable<HingeConstraint>
    {
        #region IEquatable
        
        public bool Equals(HingeConstraint other) => Handle.Equals(other.Handle);
        
        public override bool Equals(object obj) => obj is HingeConstraint other && Equals(other);
        
        public override int GetHashCode() => Handle.GetHashCode();
        
        public static bool operator ==(HingeConstraint lhs, HingeConstraint rhs) => lhs.Equals(rhs);
        
        public static bool operator !=(HingeConstraint lhs, HingeConstraint rhs) => !lhs.Equals(rhs);
        
        #endregion
        
        #region JPH_HingeConstraint
        
        public HingeConstraintSettings GetSettings() => Bindings.JPH_HingeConstraint_GetSettings(Handle);
        
        public float3 GetLocalSpacePoint1() => Bindings.JPH_HingeConstraint_GetLocalSpacePoint1(Handle);
        
        public float3 GetLocalSpacePoint2() => Bindings.JPH_HingeConstraint_GetLocalSpacePoint2(Handle);
        
        public float3 GetLocalSpaceHingeAxis1() => Bindings.JPH_HingeConstraint_GetLocalSpaceHingeAxis1(Handle);
        
        public float3 GetLocalSpaceHingeAxis2() => Bindings.JPH_HingeConstraint_GetLocalSpaceHingeAxis2(Handle);
        
        public float3 GetLocalSpaceNormalAxis1() => Bindings.JPH_HingeConstraint_GetLocalSpaceNormalAxis1(Handle);
        
        public float3 GetLocalSpaceNormalAxis2() => Bindings.JPH_HingeConstraint_GetLocalSpaceNormalAxis2(Handle);
        
        public float GetCurrentAngle() => Bindings.JPH_HingeConstraint_GetCurrentAngle(Handle);
        
        public void SetMaxFrictionTorque(float frictionTorque) => Bindings.JPH_HingeConstraint_SetMaxFrictionTorque(Handle, frictionTorque);
        
        public float GetMaxFrictionTorque() => Bindings.JPH_HingeConstraint_GetMaxFrictionTorque(Handle);
        
        public void SetMotorSettings(MotorSettings settings) => Bindings.JPH_HingeConstraint_SetMotorSettings(Handle, settings);
        
        public MotorSettings GetMotorSettings() => Bindings.JPH_HingeConstraint_GetMotorSettings(Handle);
        
        public void SetMotorState(MotorState state) => Bindings.JPH_HingeConstraint_SetMotorState(Handle, state);
        
        public MotorState GetMotorState() => Bindings.JPH_HingeConstraint_GetMotorState(Handle);
        
        public void SetTargetAngularVelocity(float angularVelocity) => Bindings.JPH_HingeConstraint_SetTargetAngularVelocity(Handle, angularVelocity);
        
        public float GetTargetAngularVelocity() => Bindings.JPH_HingeConstraint_GetTargetAngularVelocity(Handle);
        
        public void SetTargetAngle(float angle) => Bindings.JPH_HingeConstraint_SetTargetAngle(Handle, angle);
        
        public float GetTargetAngle() => Bindings.JPH_HingeConstraint_GetTargetAngle(Handle);
        
        public void SetLimits(float min, float max) => Bindings.JPH_HingeConstraint_SetLimits(Handle, min, max);
        
        public float GetLimitsMin() => Bindings.JPH_HingeConstraint_GetLimitsMin(Handle);
        
        public float GetLimitsMax() => Bindings.JPH_HingeConstraint_GetLimitsMax(Handle);
        
        public bool HasLimits() => Bindings.JPH_HingeConstraint_HasLimits(Handle);
        
        public SpringSettings GetLimitsSpringSettings() => Bindings.JPH_HingeConstraint_GetLimitsSpringSettings(Handle);
        
        public void SetLimitsSpringSettings(SpringSettings settings) => Bindings.JPH_HingeConstraint_SetLimitsSpringSettings(Handle, settings);
        
        public float3 GetTotalLambdaPosition() => Bindings.JPH_HingeConstraint_GetTotalLambdaPosition(Handle);
        
        public float2 GetTotalLambdaRotation() => Bindings.JPH_HingeConstraint_GetTotalLambdaRotation(Handle);
        
        public float GetTotalLambdaRotationLimits() => Bindings.JPH_HingeConstraint_GetTotalLambdaRotationLimits(Handle);
        
        public float GetTotalLambdaMotor() => Bindings.JPH_HingeConstraint_GetTotalLambdaMotor(Handle);
        
        #endregion
        
        #region JPH_TwoBodyConstraint
        
        public Body GetBody1() => new Body { Handle = Bindings.JPH_TwoBodyConstraint_GetBody1(Handle.Reinterpret<JPH_TwoBodyConstraint>()) };
        
        public Body GetBody2() => new Body { Handle = Bindings.JPH_TwoBodyConstraint_GetBody2(Handle.Reinterpret<JPH_TwoBodyConstraint>()) };
        
        public float4x4 GetConstraintToBody1Matrix() => Bindings.JPH_TwoBodyConstraint_GetConstraintToBody1Matrix(Handle.Reinterpret<JPH_TwoBodyConstraint>());
        
        public float4x4 GetConstraintToBody2Matrix() => Bindings.JPH_TwoBodyConstraint_GetConstraintToBody2Matrix(Handle.Reinterpret<JPH_TwoBodyConstraint>());
        
        #endregion
        
        #region JPH_Constraint
        
        public void Destroy() => Bindings.JPH_Constraint_Destroy(Handle.Reinterpret<JPH_Constraint>());
        
        public ConstraintType GetShapeType() => Bindings.JPH_Constraint_GetType(Handle.Reinterpret<JPH_Constraint>());
        
        public ConstraintSubType GetSubType() => Bindings.JPH_Constraint_GetSubType(Handle.Reinterpret<JPH_Constraint>());
        
        public uint GetConstraintPriority() => Bindings.JPH_Constraint_GetConstraintPriority(Handle.Reinterpret<JPH_Constraint>());
        
        public void SetConstraintPriority(uint priority) => Bindings.JPH_Constraint_SetConstraintPriority(Handle.Reinterpret<JPH_Constraint>(), priority);
        
        public uint GetNumVelocityStepsOverride() => Bindings.JPH_Constraint_GetNumVelocityStepsOverride(Handle.Reinterpret<JPH_Constraint>());
        
        public void SetNumVelocityStepsOverride(uint value) => Bindings.JPH_Constraint_SetNumVelocityStepsOverride(Handle.Reinterpret<JPH_Constraint>(), value);
        
        public uint GetNumPositionStepsOverride() => Bindings.JPH_Constraint_GetNumPositionStepsOverride(Handle.Reinterpret<JPH_Constraint>());
        
        public void SetNumPositionStepsOverride(uint value) => Bindings.JPH_Constraint_SetNumPositionStepsOverride(Handle.Reinterpret<JPH_Constraint>(), value);
        
        public bool GetEnabled() => Bindings.JPH_Constraint_GetEnabled(Handle.Reinterpret<JPH_Constraint>());
        
        public void SetEnabled(bool enabled) => Bindings.JPH_Constraint_SetEnabled(Handle.Reinterpret<JPH_Constraint>(), enabled);
        
        public ulong GetUserData() => Bindings.JPH_Constraint_GetUserData(Handle.Reinterpret<JPH_Constraint>());
        
        public void SetUserData(ulong userData) => Bindings.JPH_Constraint_SetUserData(Handle.Reinterpret<JPH_Constraint>(), userData);
        
        public void NotifyShapeChanged(BodyID bodyID, float3 deltaCOM) => Bindings.JPH_Constraint_NotifyShapeChanged(Handle.Reinterpret<JPH_Constraint>(), bodyID, deltaCOM);
        
        public void ResetWarmStart() => Bindings.JPH_Constraint_ResetWarmStart(Handle.Reinterpret<JPH_Constraint>());
        
        public bool IsActive() => Bindings.JPH_Constraint_IsActive(Handle.Reinterpret<JPH_Constraint>());
        
        public void SetupVelocityConstraint(float deltaTime) => Bindings.JPH_Constraint_SetupVelocityConstraint(Handle.Reinterpret<JPH_Constraint>(), deltaTime);
        
        public void WarmStartVelocityConstraint(float warmStartImpulseRatio) => Bindings.JPH_Constraint_WarmStartVelocityConstraint(Handle.Reinterpret<JPH_Constraint>(), warmStartImpulseRatio);
        
        public bool SolveVelocityConstraint(float deltaTime) => Bindings.JPH_Constraint_SolveVelocityConstraint(Handle.Reinterpret<JPH_Constraint>(), deltaTime);
        
        public bool SolvePositionConstraint(float deltaTime, float baumgarte) => Bindings.JPH_Constraint_SolvePositionConstraint(Handle.Reinterpret<JPH_Constraint>(), deltaTime, baumgarte);
        
        #endregion
        
    }
}
