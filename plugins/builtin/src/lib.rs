//! Reference host plugins compiled into the engine.
//!
//! They depend only on the public plugin SDK, the
//! same contract external plugins use.

use motioncore_plugin_sdk::{Manifest, Plugin, PluginContext, PluginKind, API_VERSION};

fn manifest(id: &str, description: &str, provides: &[&str], requires: &[&str], permissions: &[&str]) -> Manifest {
    let s = |v: &[&str]| v.iter().map(|x| x.to_string()).collect();
    Manifest {
        id: id.into(),
        version: env!("CARGO_PKG_VERSION").into(),
        api_version: API_VERSION.into(),
        kind: PluginKind::Builtin,
        description: description.into(),
        entrypoint: None,
        sha256: None,
        provides: s(provides),
        requires: s(requires),
        permissions: s(permissions),
    }
}

/// Robot profile for a generic chain of independent joints.
pub struct SingleAxisProfile;

impl Plugin for SingleAxisProfile {
    fn manifest(&self) -> Manifest {
        manifest(
            "org.motioncore.robot.single_axis",
            "Generic joint-space robot profile (one or more independent axes)",
            &["robot.profile"],
            &["robot.joint_model"],
            &["robot.read_geometry"],
        )
    }
    fn activate(&mut self, ctx: &PluginContext) -> Result<(), String> {
        if ctx.axis_ids.is_empty() {
            return Err("robot has no axes".into());
        }
        Ok(())
    }
}

/// Step/dir actuator adapter.
pub struct StepDirActuator;

impl Plugin for StepDirActuator {
    fn manifest(&self) -> Manifest {
        manifest(
            "org.motioncore.actuator.step_dir",
            "Step/direction stepper driver adapter",
            &["actuator.step_dir"],
            &["mcu.stepper"],
            &["robot.read_geometry"],
        )
    }
    fn activate(&mut self, _: &PluginContext) -> Result<(), String> {
        Ok(())
    }
}

/// Joint-space identity kinematics: the pose of a joint-space robot is its
/// joint vector, so forward and inverse kinematics are the identity.
#[derive(Default)]
pub struct JointIdentityKinematics {
    /// Joint count while active.
    joints: Option<usize>,
}

impl JointIdentityKinematics {
    /// Joint count while the plugin is active.
    pub fn joints(&self) -> Option<usize> {
        self.joints
    }
}

impl Plugin for JointIdentityKinematics {
    fn manifest(&self) -> Manifest {
        manifest(
            "org.motioncore.kinematics.joint_identity",
            "Identity forward/inverse kinematics for joint-space robots",
            &["kinematics.fk", "kinematics.ik"],
            &["robot.profile"],
            &["robot.read_geometry"],
        )
    }
    fn activate(&mut self, ctx: &PluginContext) -> Result<(), String> {
        self.joints = Some(ctx.axis_ids.len());
        Ok(())
    }
    fn deactivate(&mut self) {
        self.joints = None;
    }
}

/// All builtin plugins.
pub fn all() -> Vec<Box<dyn Plugin>> {
    vec![Box::new(SingleAxisProfile), Box::new(StepDirActuator), Box::new(JointIdentityKinematics::default())]
}

#[cfg(test)]
mod tests {
    use motioncore_plugin_runtime::Registry;
    use motioncore_plugin_sdk::{PluginContext, CORE_CAPABILITIES};

    #[test]
    fn builtins_validate_and_activate() {
        for p in super::all() {
            p.manifest().validate().unwrap();
        }
        let mut r = Registry::new(CORE_CAPABILITIES.iter().copied());
        for p in super::all() {
            r.add_builtin(p);
        }
        let ctx = PluginContext { robot_name: "t".into(), axis_ids: vec!["j1".into()] };
        assert!(r.start(&ctx).is_empty(), "{:?}", r.info());
        assert!(r.is_active("org.motioncore.robot.single_axis"));
        assert!(r.has_capability("kinematics.ik"));
    }
}
