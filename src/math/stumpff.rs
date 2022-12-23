use std::cmp::Ordering;

pub fn stumpff_c(x: f64) -> [f64; 4] {
    [c0(x), c1(x), c2(x), c3(x)]
}

#[allow(non_snake_case)]
pub fn stumpff_G(beta: f64, s: f64) -> [f64; 4] {
    // the kth entry should be s^k c_k(beta s^2)
    let mut output = stumpff_c(beta * s * s);
    for (k, value) in output.iter_mut().enumerate() {
        *value *= s.powi(k as i32);
    }

    output
}

fn compare_to_zero(x: f64) -> Ordering {
    match x.partial_cmp(&0.0) {
        Some(x) => x,
        None => panic!("Got a weird float: {:?}", x),
    }
}

pub fn c0(x: f64) -> f64 {
    match compare_to_zero(x) {
        Ordering::Greater => x.sqrt().cos(),
        Ordering::Less => (-x).sqrt().cosh(),
        Ordering::Equal => 1.0,
    }
}

pub fn c1(x: f64) -> f64 {
    match compare_to_zero(x) {
        Ordering::Greater => x.sqrt().sin() / x.sqrt(),
        Ordering::Less => (-x).sqrt().sinh() / (-x).sqrt(),
        Ordering::Equal => 1.0,
    }
}

pub fn c2(x: f64) -> f64 {
    match compare_to_zero(x) {
        // 1 - cos u = 2 sin^2(u/2)
        Ordering::Greater => 2.0 * (x.sqrt() / 2.0).sin().powi(2) / x,
        // 1 - cosh u = -2 sinh^2(u/2)
        Ordering::Less => -2.0 * ((-x).sqrt() / 2.0).sinh().powi(2) / x,
        Ordering::Equal => 0.5,
    }
}

// Computed via the stumpff binary
const C3_CHEBYSHEV: [f64; 9] = [
    1.6676588241065263e-1,
    -8.335400232645692e-3,
    9.921887561900632e-5,
    -6.889831660341532e-7,
    3.1316569342984595e-9,
    -1.0037209903903158e-11,
    2.3897900455039615e-14,
    -4.392970771382075e-17,
    6.422446836919863e-20,
];

pub fn c3(x: f64) -> f64 {
    // Check if we're close to the origin. If so, do fancy Chebyshev math,
    // to avoid catastrophic cancellation.
    // Otherwise we're good with the naive formula.
    if x.abs() < 1.0 {
        evaluate_chebyshev(x, &C3_CHEBYSHEV)
    } else {
        (1.0 - c1(x)) / x
    }
}

pub fn evaluate_chebyshev(x: f64, coeffs: &[f64]) -> f64 {
    // uses clenshaw's algorithm to evaluate a sum of chebyshev polynomials.
    // apparently it's nice and stable

    let n = coeffs.len() - 1; // max degree

    let mut b_k_plus_2 = 0.0;
    let mut b_k_plus_1 = 0.0;

    // recurrence is b_k = a_k + 2x b_(k+1) - b_(k+2)
    for k in (1..=n).rev() {
        let b_k = coeffs[k] + 2.0 * x * b_k_plus_1 - b_k_plus_2;

        // shift down
        b_k_plus_2 = b_k_plus_1;
        b_k_plus_1 = b_k;
    }

    // we just finished k = 1, so now k = 0 here
    // and we have b_1 and b_2
    coeffs[0] + x * b_k_plus_1 - b_k_plus_2
}
