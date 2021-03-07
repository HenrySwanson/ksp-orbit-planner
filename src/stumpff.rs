use std::cmp::Ordering;

pub fn stumpff_c(x: f64) -> [f64; 4] {
    [c0(x), c1(x), c2(x), c3(x)]
}

#[allow(non_snake_case)]
pub fn stumpff_G(beta: f64, s: f64) -> [f64; 4] {
    // the kth entry should be s^k c_k(beta s^2)
    let mut output = stumpff_c(beta * s * s);
    for k in 0..4 {
        output[k] *= s.powi(k as i32);
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

// Computed via get_coeff_of_stumpff
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
        chebyshev::evaluate_chebyshev(x, &C3_CHEBYSHEV)
    } else {
        (1.0 - c1(x)) / x
    }
}

// --- testing stumpff functions ---
#[allow(dead_code)]
pub mod plotting {
    use super::*;
    use plotters::prelude::*;

    pub fn draw_plots() {
        let scale = 1e-7;
        draw_plot(
            "plots/c2-naive.png",
            |x| (naive_c2(x * scale) - 0.5) / scale,
            200,
        )
        .unwrap();
        draw_plot(
            "plots/c2.png",
            |x| (super::c2(x * scale) - 0.5) / scale,
            200,
        )
        .unwrap();

        let scale = 1e-7;
        let sixth = 1.0 / 6.0;
        draw_plot(
            "plots/c3-naive.png",
            |x| (naive_c3(x * scale) - sixth) / scale,
            200,
        )
        .unwrap();
        draw_plot(
            "plots/c3.png",
            |x| (super::c3(x * scale) - sixth) / scale,
            200,
        )
        .unwrap();
    }
    fn draw_plot(
        name: &str,
        func: impl Fn(f64) -> f64,
        n_points: usize,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let root = BitMapBackend::new(name, (640, 640)).into_drawing_area();
        root.fill(&WHITE)?;
        let mut chart = ChartBuilder::on(&root)
            .margin(5)
            .x_label_area_size(30)
            .y_label_area_size(30)
            .build_cartesian_2d(-1.0f32..1.0f32, -1.0f32..1.0f32)?;

        chart.configure_mesh().draw()?;

        chart.draw_series(LineSeries::new(
            (0..=n_points)
                .map(|i| (2 * i as i32 - n_points as i32) as f64 / (n_points as f64))
                .map(|x| (x as f32, func(x) as f32)),
            &RED,
        ))?;

        Ok(())
    }

    fn naive_c2(x: f64) -> f64 {
        match x.partial_cmp(&0.0).unwrap() {
            Ordering::Equal => 0.5,
            _ => (1.0 - c0(x)) / x,
        }
    }

    fn naive_c3(x: f64) -> f64 {
        match x.partial_cmp(&0.0).unwrap() {
            Ordering::Greater => (1.0 - c1(x)) / x,
            Ordering::Less => (1.0 - c1(x)) / x,
            Ordering::Equal => 1.0 / 6.0,
        }
    }
}

#[allow(dead_code)]
pub mod chebyshev {

    fn get_coeff_of_xn(n: usize, k: usize) -> f64 {
        // x^n only has coefficients on T_0 through T_n
        if k > n {
            return 0.0;
        }

        // x^(odd) has zero coefficients on T_(even)
        // and vice versa
        if (n - k) % 2 != 0 {
            return 0.0;
        }

        // Non-zero coefficients are computed like so:
        let j = (n - k) / 2;
        let mut binomial = 1.0; // n choose j
        for i in 0..j {
            binomial *= (n - i) as f64 / (j - i) as f64;
        }

        let coeff = binomial / 2.0_f64.powi(n as i32 - 1);

        // T_0 is a special case
        if k == 0 {
            coeff / 2.0
        } else {
            coeff
        }
    }

    fn is_negligible_against(epsilon: f64, value: f64) -> bool {
        // This is a good stopping criterion for series computation
        epsilon != 0.0 && value + epsilon == value
    }

    fn get_coeff_of_stumpff(n: usize, k: usize) -> f64 {
        let mut total = 0.0;

        // The coefficients start at a_0 = 1/n!
        // and are, in general, a_i = (-1)^i / (n + 2i)!
        let mut coeff_i = 1.0;
        for j in 1..=n {
            coeff_i /= j as f64;
        }

        for i in 0.. {
            let delta = coeff_i * get_coeff_of_xn(i, k);

            let old_total = total;
            total += delta;

            // We have to check that we didn't just try to add 0, but otherwise,
            // this tells us if we've hit machine precision.
            if is_negligible_against(delta, total) {
                break;
            }

            // Increment to a_(i+1) = 1/(n + 2i + 2)!
            coeff_i /= ((n + 2 * i + 1) * (n + 2 * i + 2)) as f64;
            coeff_i = -coeff_i; // flip sign
        }

        total
    }

    pub fn generate_stumpff_coefficients(n: usize, bound: f64) {
        for k in 0.. {
            let a_k = get_coeff_of_stumpff(n, k);
            println!("{}th coefficient of c_{} = {:e}", k, n, a_k);

            // We stop when the coefficient is too small to affect anything anymore.
            // That happens when (a_k + c_n(X)) = c_n(X), for all X in [-1, 1].
            // That's what the `bound` parameter is for. We just pass in the biggest
            // value (in magnitude) that c_n achieves on [-1, 1], or just something
            // bigger than that.
            if is_negligible_against(a_k, bound) {
                break;
            }
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
}
