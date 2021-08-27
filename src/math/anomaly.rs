use crate::math::root_finding::{find_root_bracket, newton_plus_bisection};

pub fn mean_to_eccentric(mean_anomaly: f64, e: f64) -> f64 {
    assert!(e < 1.0);

    // This doesn't have a closed form, so let's do some rootfinding
    let kepler = |x: f64| -> f64 { x - e * x.sin() - mean_anomaly };
    let kepler_der = |x: f64| -> f64 { 1.0 - e * x.cos() };

    let bracket = find_root_bracket(kepler, mean_anomaly, e + 0.1, 100);
    newton_plus_bisection(|x| (kepler(x), kepler_der(x)), bracket, 100)
}

pub fn eccentric_to_mean(eccentric_anomaly: f64, e: f64) -> f64 {
    assert!(e < 1.0);
    eccentric_anomaly - e * eccentric_anomaly.sin()
}

#[inline]
fn eccentric_factor(e: f64) -> f64 {
    ((1.0 - e) / (1.0 + e)).sqrt()
}

pub fn eccentric_to_true(eccentric_anomaly: f64, e: f64) -> f64 {
    // We have that tan(E/2) = sqrt((1-e)/(1+e)) * tan(theta/2)
    let tan_half_ecc = (eccentric_anomaly / 2.0).tan();
    let tan_half_theta = tan_half_ecc / eccentric_factor(e);
    2.0 * tan_half_theta.atan()

    // TODO: watch out for wrapping at 2pi!
}

pub fn true_to_eccentric(true_anomaly: f64, e: f64) -> f64 {
    let tan_half_theta = (true_anomaly / 2.0).tan();
    let tan_half_ecc = tan_half_theta * eccentric_factor(e);
    2.0 * tan_half_ecc.atan()
}

#[inline]
fn hyperbolic_factor(e: f64) -> f64 {
    ((e - 1.0) / (e + 1.0)).sqrt()
}

pub fn hyperbolic_to_true(hyperbolic_anomaly: f64, e: f64) -> f64 {
    // We have that tanh(H/2) = sqrt((e-1)/(e+1)) tan(theta/2)
    let tanh_half_hyp = (hyperbolic_anomaly / 2.0).tanh();
    let tan_half_theta = tanh_half_hyp / hyperbolic_factor(e);
    2.0 * tan_half_theta.atan()
}

pub fn true_to_hyperbolic(true_anomaly: f64, e: f64) -> f64 {
    let tan_half_theta = (true_anomaly / 2.0).tan();
    let tanh_half_hyp = tan_half_theta * hyperbolic_factor(e);
    2.0 * tanh_half_hyp.atanh()
}

pub fn parabolic_to_true(parabolic_anomaly: f64) -> f64 {
    // We have that D = tan (theta/2)
    2.0 * parabolic_anomaly.atan()
}

pub fn true_to_parabolic(true_anomaly: f64) -> f64 {
    (true_anomaly / 2.0).tan()
}

pub fn eccentric_to_universal(eccentric_anomaly: f64, energy: f64) -> f64 {
    eccentric_anomaly / (-2.0 * energy).sqrt()
}

pub fn universal_to_eccentric(universal_anomaly: f64, energy: f64) -> f64 {
    universal_anomaly * (-2.0 * energy).sqrt()
}

pub fn hyperbolic_to_universal(hyperbolic_anomaly: f64, energy: f64) -> f64 {
    hyperbolic_anomaly / (2.0 * energy).sqrt()
}

pub fn universal_to_hyperbolic(universal_anomaly: f64, energy: f64) -> f64 {
    universal_anomaly * (2.0 * energy).sqrt()
}

pub fn parabolic_to_universal(parabolic_anomaly: f64, angular_momentum: f64, mu: f64) -> f64 {
    parabolic_anomaly * angular_momentum / mu
}

pub fn universal_to_parabolic(universal_anomaly: f64, angular_momentum: f64, mu: f64) -> f64 {
    universal_anomaly / angular_momentum * mu
}

pub fn mean_to_true(mean_anomaly: f64, ecc: f64) -> f64 {
    eccentric_to_true(mean_to_eccentric(mean_anomaly, ecc), ecc)
}

pub fn true_to_mean(true_anomaly: f64, ecc: f64) -> f64 {
    eccentric_to_mean(true_to_eccentric(true_anomaly, ecc), ecc)
}
