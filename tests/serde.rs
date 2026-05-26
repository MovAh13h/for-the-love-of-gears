#![cfg(feature = "serde")]

use for_the_love_of_gears::{
    gear::Gear,
    helical::{HelicalGear, HelixHand},
    scene::{AnyGear, Direction},
    GearError,
};

fn roundtrip<T: serde::Serialize + for<'de> serde::Deserialize<'de> + std::fmt::Debug + PartialEq>(
    value: &T,
) {
    let json = serde_json::to_string(value).expect("serialize");
    let back: T = serde_json::from_str(&json).expect("deserialize");
    assert_eq!(value, &back);
}

#[test]
fn gear_roundtrip() {
    let g = Gear::builder().module(2.0).teeth(20).build().unwrap();
    roundtrip(&g);
}

#[test]
fn gear_nonstandard_pressure_angle_roundtrip() {
    let g = Gear::builder()
        .module(2.5)
        .teeth(17)
        .pressure_angle(14.5)
        .build()
        .unwrap();
    roundtrip(&g);
}

#[test]
fn helical_gear_roundtrip() {
    let g = HelicalGear::builder()
        .module(2.0)
        .teeth(20)
        .helix_angle(20.0)
        .helix_hand(HelixHand::Right)
        .build()
        .unwrap();
    roundtrip(&g);
}

#[test]
fn helical_gear_with_face_width_roundtrip() {
    let g = HelicalGear::builder()
        .module(2.0)
        .teeth(30)
        .helix_angle(25.0)
        .helix_hand(HelixHand::Left)
        .face_width(40.0)
        .build()
        .unwrap();
    roundtrip(&g);
}

#[test]
fn helix_hand_roundtrip() {
    roundtrip(&HelixHand::Left);
    roundtrip(&HelixHand::Right);
}

#[test]
fn any_gear_spur_roundtrip() {
    let g = AnyGear::from(Gear::builder().module(2.0).teeth(20).build().unwrap());
    roundtrip(&g);
}

#[test]
fn any_gear_helical_roundtrip() {
    let g = AnyGear::from(
        HelicalGear::builder()
            .module(2.0)
            .teeth(20)
            .helix_angle(20.0)
            .helix_hand(HelixHand::Right)
            .build()
            .unwrap(),
    );
    roundtrip(&g);
}

#[test]
fn direction_roundtrip() {
    roundtrip(&Direction::Clockwise);
    roundtrip(&Direction::CounterClockwise);
}

#[test]
fn gear_error_roundtrip() {
    roundtrip(&GearError::ModuleRequired);
    roundtrip(&GearError::TeethTooFew);
    roundtrip(&GearError::ModuleMustBePositive);
}
