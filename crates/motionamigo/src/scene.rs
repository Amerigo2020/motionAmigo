//! Scene format v0.1, shared with the sister project spatialAmigo.
//!
//! A scene is a JSON document in meters, with a right-handed world frame whose z axis points up.
//! Every object is an oriented box: `center` is the box center, `size` holds the full edge
//! lengths and `yaw` is the rotation about the world z axis in radians. The optional `front`
//! vector and `viewpoint` are used by spatialAmigo to resolve spatial language and are ignored
//! by the planner. The JSON Schema lives in `schema/scene-v0.1.json`.
//!
//! ```
//! use motionamigo::scene::Scene;
//! let json = r#"{"version": "0.1", "frame": "world", "objects": [
//!     {"id": "box_1", "label": "box", "center": [0.5, 0.0, 0.1], "size": [0.2, 0.2, 0.2], "yaw": 0.0}
//! ]}"#;
//! let scene = Scene::from_json(json).unwrap();
//! assert_eq!(scene.objects[0].id, "box_1");
//! ```

use serde::{Deserialize, Serialize};

/// The only scene format version this crate understands.
pub const SCENE_FORMAT_VERSION: &str = "0.1";

/// Errors that can occur while loading a scene.
#[derive(Debug, thiserror::Error)]
pub enum SceneError {
    /// The document is not valid JSON or does not match the expected structure.
    #[error("invalid scene JSON: {0}")]
    Json(#[from] serde_json::Error),
    /// The scene declares a version other than [`SCENE_FORMAT_VERSION`].
    #[error("unsupported scene version {found:?}, expected {expected:?}")]
    Version {
        /// Version found in the document.
        found: String,
        /// Version supported by this crate.
        expected: &'static str,
    },
    /// The scene is structurally valid JSON but violates a semantic rule.
    #[error("invalid scene: {0}")]
    Invalid(String),
}

/// A complete scene document.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Scene {
    /// Format version, always `"0.1"` for now.
    pub version: String,
    /// Name of the frame all coordinates are expressed in, usually `"world"`.
    pub frame: String,
    /// Objects in the scene.
    pub objects: Vec<SceneObject>,
    /// Optional observer viewpoint (ignored by the planner).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub viewpoint: Option<Viewpoint>,
}

/// An object in the scene, approximated as an oriented box.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SceneObject {
    /// Unique identifier, for example `"mug_1"`.
    pub id: String,
    /// Category label, for example `"mug"`.
    pub label: String,
    /// Box center in meters.
    pub center: [f64; 3],
    /// Full edge lengths in meters.
    pub size: [f64; 3],
    /// Rotation about the world z axis in radians.
    pub yaw: f64,
    /// Optional intrinsic front direction in the object frame (ignored by the planner).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub front: Option<[f64; 3]>,
}

/// Optional observer pose used by spatialAmigo.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Viewpoint {
    /// Camera position in meters.
    pub position: [f64; 3],
    /// Point the camera looks at.
    pub look_at: [f64; 3],
}

impl Scene {
    /// Parses and validates a scene from a JSON string.
    pub fn from_json(json: &str) -> Result<Self, SceneError> {
        let scene: Scene = serde_json::from_str(json)?;
        scene.validate()?;
        Ok(scene)
    }

    /// Reads, parses and validates a scene file.
    pub fn from_path(path: impl AsRef<std::path::Path>) -> Result<Self, SceneError> {
        let text = std::fs::read_to_string(path.as_ref()).map_err(|e| {
            SceneError::Invalid(format!("cannot read {}: {e}", path.as_ref().display()))
        })?;
        Self::from_json(&text)
    }

    /// Serializes the scene to pretty-printed JSON.
    pub fn to_json(&self) -> String {
        serde_json::to_string_pretty(self).expect("scene serialization cannot fail")
    }

    /// Looks up an object by id.
    pub fn object(&self, id: &str) -> Option<&SceneObject> {
        self.objects.iter().find(|o| o.id == id)
    }

    /// Checks the semantic rules that JSON deserialization alone cannot enforce.
    pub fn validate(&self) -> Result<(), SceneError> {
        if self.version != SCENE_FORMAT_VERSION {
            return Err(SceneError::Version {
                found: self.version.clone(),
                expected: SCENE_FORMAT_VERSION,
            });
        }
        if self.frame.is_empty() {
            return Err(SceneError::Invalid("frame must not be empty".into()));
        }
        let mut ids = std::collections::HashSet::new();
        for o in &self.objects {
            if o.id.is_empty() {
                return Err(SceneError::Invalid("object id must not be empty".into()));
            }
            if !ids.insert(o.id.as_str()) {
                return Err(SceneError::Invalid(format!(
                    "duplicate object id {:?}",
                    o.id
                )));
            }
            let finite = o.center.iter().chain(o.size.iter()).all(|v| v.is_finite());
            if !finite || !o.yaw.is_finite() {
                return Err(SceneError::Invalid(format!(
                    "object {:?} has non-finite values",
                    o.id
                )));
            }
            if o.size.iter().any(|&s| s <= 0.0) {
                return Err(SceneError::Invalid(format!(
                    "object {:?} must have strictly positive size",
                    o.id
                )));
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TABLETOP: &str = include_str!("../../../examples/scenes/tabletop.json");

    #[test]
    fn parses_tabletop_example() {
        let scene = Scene::from_json(TABLETOP).unwrap();
        assert_eq!(scene.objects.len(), 7);
        let mug = scene.object("mug_2").unwrap();
        assert_eq!(mug.center, [0.75, 0.38, 0.45]);
        assert_eq!(
            scene.object("laptop_1").unwrap().front,
            Some([1.0, 0.0, 0.0])
        );
        assert!(scene.viewpoint.is_some());
    }

    #[test]
    fn roundtrips_through_json() {
        let scene = Scene::from_json(TABLETOP).unwrap();
        let again = Scene::from_json(&scene.to_json()).unwrap();
        assert_eq!(scene, again);
    }

    #[test]
    fn rejects_wrong_version() {
        let bad = TABLETOP.replace("\"0.1\"", "\"0.2\"");
        assert!(matches!(
            Scene::from_json(&bad),
            Err(SceneError::Version { .. })
        ));
    }

    #[test]
    fn rejects_duplicate_ids_and_bad_sizes() {
        let dup = r#"{"version":"0.1","frame":"world","objects":[
            {"id":"a","label":"x","center":[0,0,0],"size":[1,1,1],"yaw":0},
            {"id":"a","label":"x","center":[0,0,0],"size":[1,1,1],"yaw":0}]}"#;
        assert!(Scene::from_json(dup).is_err());
        let neg = r#"{"version":"0.1","frame":"world","objects":[
            {"id":"a","label":"x","center":[0,0,0],"size":[1,0,1],"yaw":0}]}"#;
        assert!(Scene::from_json(neg).is_err());
    }
}
