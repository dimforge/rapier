# 03 — `mjcf-rs` Parser Crate Design

## Goals

1. Pure-Rust XML → typed AST for MJCF, with no rapier dependency.
2. Resolve all "preprocessing" the loader shouldn't have to think about:
   - `<include>` inlining (recursive, with cycle detection)
   - `<default>` class inheritance (every concrete element ends up with all
     its attributes filled in from the default chain)
   - `<compiler angle>` / `<compiler eulerseq>` normalisation (every angle
     coming out of the AST is in radians; every Euler sequence is canonical)
3. Light, no surprises. The output AST mirrors MJCF 1:1 in element names so
   users who know the spec can navigate it.

## Non-goals

- No serialization (MJCF write-back) on the v1 roadmap. Possible later.
- No lossy "model compilation" (we don't fuse bodies, balance inertia,
  derive masses from geoms — that's the loader's job, since it depends on
  rapier knowledge).
- No simulation. This crate is purely descriptive.

## Dependencies

| Dep             | Use                           |
| --------------- | ----------------------------- |
| `quick-xml`     | Streaming XML reader.         |
| `thiserror`     | Error type.                   |
| `roxmltree`     | (Alternative — DOM-style API; possibly nicer for `<default>` traversal. Decide during implementation.) |
| `glam` / `nalgebra` | **Not used.** The parser exposes plain `[f64; 3]` / `[f64; 4]` arrays so it has no math library opinion. The loader does the math conversion. |

## Public surface (sketch)

```rust
pub struct Model {
    pub name: Option<String>,
    pub compiler: Compiler,        // resolved (post-default-inheritance)
    pub option: Option<Option_>,   // raw
    pub assets: Assets,
    pub world: Body,               // implicit "world" body holding worldbody children
    pub contact: Contact,
    pub equality: Vec<Equality>,
    pub tendon: Vec<Tendon>,
    pub actuator: Vec<Actuator>,
    pub sensor: Vec<Sensor>,
    pub keyframes: Vec<Keyframe>,
    pub custom: Custom,
}

pub struct Body {
    pub name: Option<String>,
    pub pose: Pose,                // axis-angle / quat / euler all resolved into one canonical form
    pub mocap: bool,
    pub gravcomp: f64,
    pub childclass: Option<String>,
    pub inertial: Option<Inertial>,
    pub joints: Vec<Joint>,        // preserves order; freejoint is encoded as Joint { type: Free, .. }
    pub geoms: Vec<Geom>,
    pub sites: Vec<Site>,
    pub bodies: Vec<Body>,         // children
    pub frames: Vec<Frame>,        // <frame> children — preserved or pre-folded into bodies (TBD during impl)
    pub user: Vec<f64>,
}

pub struct Joint {
    pub name: Option<String>,
    pub class: Option<String>,
    pub type_: JointType,
    pub pos: [f64; 3],
    pub axis: [f64; 3],
    pub limited: Tristate,
    pub range: Option<[f64; 2]>,
    pub stiffness: f64,
    pub damping: f64,
    pub springref: f64,
    pub springdamper: Option<[f64; 2]>,
    pub armature: f64,
    pub frictionloss: f64,
    pub ref_: f64,
    pub margin: f64,
    pub user: Vec<f64>,
}

pub enum JointType { Hinge, Slide, Ball, Free }
pub enum Tristate { True, False, Auto }

pub struct Geom { /* ... full attribute set ... */ }
pub struct Inertial { /* ... */ }
pub struct Site { /* ... */ }
// etc.
```

`Pose` here is the parser's small wrapper: `{ pos: [f64;3], rot: PoseRotation }`
where `PoseRotation` is one of `Quat`, `AxisAngle`, `XyAxes`, `ZAxis`, `Euler`.
The loader decides how to convert it to its own math types — we don't push
`glam`/`nalgebra` opinions into the parser.

## Pipeline

```
read_to_string(path)
   ↓
quick-xml streaming pass: build a raw element tree
   ↓
resolve <include>:
   • recurse on each <include file="..."> using the parent file's directory
   • detect cycles via a visiting-set keyed by canonicalised path
   • inline children in place
   ↓
collect <compiler> attributes (file-level config)
   ↓
build <default> tree:
   • walk all <default> elements, build a class hierarchy
   • for each class, store the per-element-type defaults
   ↓
resolve every concrete instance:
   • for each <body>, <joint>, <geom>, <site>, ... element:
       1. start from defaults of the class chain (class → parent class → … → main)
       2. overlay the element's own attributes
       3. respect childclass propagation through <body>
   ↓
normalise angles:
   • if compiler.angle == "degree", convert all angle attributes in joint/range,
     <body euler>, <inertial euler>, <geom euler>, <site euler>, etc.
   • emit Euler triples in the order specified by compiler.eulerseq
     (each triple gets its rotation order tag)
   ↓
return Model
```

## Public API

```rust
impl Model {
    pub fn from_str(xml: &str, base_dir: &Path) -> Result<Self, ParseError>;
    pub fn from_file(path: impl AsRef<Path>) -> Result<Self, ParseError>;
}
```

`base_dir` lets `from_str` resolve `<include file="…">` and `meshdir`-relative
paths even when the source isn't read from disk.

## Errors

```rust
pub enum ParseError {
    Xml(quick_xml::Error),
    Io { path: PathBuf, source: io::Error },
    UnknownElement { tag: String, path: ElementPath },
    UnknownAttribute { tag: String, attr: String, path: ElementPath },
    BadAttributeValue { tag: String, attr: String, value: String, expected: &'static str, path: ElementPath },
    UnresolvedClass { class: String, path: ElementPath },
    IncludeCycle { trace: Vec<PathBuf> },
    UnsupportedFeature { feature: &'static str, path: ElementPath, hint: &'static str },
}
```

`ElementPath` is a breadcrumb trail for error messages (`"mujoco>worldbody>body[robot]>joint[shoulder]"`).

## Test fixtures

We'll snapshot a handful of public MJCFs in the crate's `tests/fixtures/`
directory and parse them in unit tests. Candidate files (all small, MIT/Apache
licensed):

- `humanoid.xml` from MuJoCo's stock examples
- `cartpole.xml`
- A pair of files that exercise `<include>` and `<default>` chains
- A file with `compiler/angle="radian"` to exercise the unit-conversion path

Tests assert (a) the parse succeeds, (b) selected element counts/values
match expected, and (c) for `<default>` cases the resolved instance has the
inherited attributes baked in.
