pub fn mean_to_ecc(mean_anomaly: f64, ecc: f64) -> f64 {
    // This doesn't have a closed form, so let's do some rootfinding
    use crate::root_finding::{find_root_bracket, newton_plus_bisection};
    let kepler = |x: f64| -> f64 { x - ecc * x.sin() - mean_anomaly };
    let kepler_der = |x: f64| -> f64 { 1.0 - ecc * x.cos() };

    let bracket = find_root_bracket(kepler, mean_anomaly, ecc + 0.1);
    newton_plus_bisection(|x| (kepler(x), kepler_der(x)), bracket, 100)
}

pub fn ecc_to_mean(ecc_anomaly: f64, ecc: f64) -> f64 {
    ecc_anomaly - ecc * ecc_anomaly.sin()
}

fn ecc_factor(ecc: f64) -> f64 {
    ((1.0 + ecc) / (1.0 - ecc)).sqrt()
}

pub fn ecc_to_true(ecc_anomaly: f64, ecc: f64) -> f64 {
    // We have that tan(theta/2) = sqrt((1+e)/(1-e)) * tan(E/2)
    let tan_half_ecc = (ecc_anomaly / 2.0).tan();
    let tan_half_theta = tan_half_ecc * ecc_factor(ecc);
    2.0 * tan_half_theta.atan()

    // TODO: watch out for wrapping at 2pi!
}

pub fn true_to_ecc(true_anomaly: f64, ecc: f64) -> f64 {
    let tan_half_theta = (true_anomaly / 2.0).tan();
    let tan_half_ecc = tan_half_theta / ecc_factor(ecc);
    2.0 * tan_half_ecc.atan()
}

pub fn mean_to_true(mean_anomaly: f64, ecc: f64) -> f64 {
    ecc_to_true(mean_to_ecc(mean_anomaly, ecc), ecc)
}

pub fn true_to_mean(true_anomaly: f64, ecc: f64) -> f64 {
    ecc_to_mean(true_to_ecc(true_anomaly, ecc), ecc)
}
