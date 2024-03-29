use plotters::prelude::*;
use rust_ksp::math::stumpff::{c0, c1, c2, c3};

pub fn main() {
    draw_plot(
        "plots/c2-naive.png",
        |x| naive_stumpff(2, x),
        200,
        (0.0, 0.5),
        1e-7,
    );
    draw_plot("plots/c2.png", c2, 200, (0.0, 0.5), 1e-7);

    draw_plot(
        "plots/c3-naive.png",
        |x| naive_stumpff(3, x),
        200,
        (0.0, 1.0 / 6.0),
        1e-7,
    );

    draw_plot("plots/c3.png", c3, 200, (0.0, 1.0 / 6.0), 1e-7);

    // Print some useful coefficients
    println!("Computing c3; use this in your code");
    generate_stumpff_coefficients(3, naive_stumpff(3, 1.0));
    println!(
        "Computing c4 and c5; check this against NASA values: \
        https://ntrs.nasa.gov/api/citations/19670018315/downloads/19670018315.pdf"
    );
    generate_stumpff_coefficients(4, naive_stumpff(4, 1.0));
    generate_stumpff_coefficients(5, naive_stumpff(5, 1.0));
}

/// Draw a plot of the given function, on the specified domain (center and
/// scale). Note: because we're working at the limits of f64, but plotter uses
/// f32 to draw, we have to rescale and recenter the datapoints before passing
/// them to plotter. This means the "origin" on the graph is not actually the
/// origin.
fn draw_plot(
    name: &str,
    func: impl Fn(f64) -> f64,
    n_points: usize,
    center: (f64, f64),
    scale: f64,
) {
    let root = BitMapBackend::new(name, (640, 640)).into_drawing_area();
    root.fill(&WHITE).unwrap();

    let mut chart = ChartBuilder::on(&root)
        .caption(
            format!(
                "Centered at ({}, {}), with scale {}",
                center.0, center.1, scale
            ),
            ("sans-serif", 24).into_font(),
        )
        .margin(5)
        .x_label_area_size(30)
        .y_label_area_size(30)
        .build_cartesian_2d(-1.0f32..1.0f32, -1.0f32..1.0f32)
        .unwrap();

    chart.configure_mesh().draw().unwrap();

    chart
        .draw_series(LineSeries::new(
            (0..=n_points)
                // Convert to u: [0, 1]
                .map(|i| i as f64 / n_points as f64)
                // Convert to actual x coordinates
                .map(|u| center.0 + (2.0 * u - 1.0) * scale)
                // Compute points
                .map(|x| (x, func(x)))
                // Recenter at the origin because f32 isn't precise enough
                .map(|(x, y)| ((x - center.0) / scale, (y - center.1) / scale))
                // Convert to f32 for drawing
                .map(|(x, y)| (x as f32, y as f32)),
            &RED,
        ))
        .unwrap();
}

fn float_factorial(n: usize) -> f64 {
    (1..=n).map(|k| k as f64).product()
}

fn naive_stumpff(n: usize, x: f64) -> f64 {
    // Avoid dividing by 0 please
    if x == 0.0 {
        return 1.0 / float_factorial(n);
    }

    match n {
        0 => c0(x),
        1 => c1(x),
        n => (float_factorial(n - 2) - naive_stumpff(n - 2, x)) / x,
    }
}

/// Prints the coefficients of the kth stumpff function. `bound` is used to
/// estimate when to stop generating coefficients. Specifically, we should stop
/// when a_k + c_n(X) is indistinguishable from c_n(X), where X is in our domain
/// of approximation [-1, 1].
///
/// So if `bound` is a lower bound for c_n on that interval, we can use it to
/// guarantee that a_k is too small to affect the outcome. And hopefully that
/// means a_k + a_{k+1} + ... is too small as well.
///
/// Note that we use the convention with the smaller a_0, so we don't have to
/// multiply the first coefficient by 1/2 during the evaluation.
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

#[allow(clippy::float_cmp)]
fn is_negligible_against(epsilon: f64, value: f64) -> bool {
    // This is a good stopping criterion for series computation
    epsilon != 0.0 && value + epsilon == value
}

/// This gives the kth coefficient in the Chebyshev series for the nth
/// Stumpff function.
fn get_coeff_of_stumpff(n: usize, k: usize) -> f64 {
    // The Stumpff function has a straightforward Taylor series:
    //   a_i = (-1)^i / (n + 2i)!
    // For each term in the Taylor series, we take the kth Chebyshev
    // coefficient of x^i, and multiply by a_i. Taking the infinite sum,
    // we would get the kth Chebyshev coefficient of c_n.

    // Let's compute a_0 first, and then modify it to get subsequent a_i.
    let mut coeff_i = 1.0;
    for j in 1..=n {
        coeff_i /= j as f64;
    }

    let mut total = 0.0;
    for i in 0.. {
        let delta = coeff_i * get_coeff_of_xn(i, k);

        total += delta;

        // We have to check that we didn't just try to add 0, but otherwise,
        // this tells us if we've hit machine precision.
        if is_negligible_against(delta, total) {
            break;
        }

        // Increment coeff_i from a_i to a_(i+1)
        coeff_i /= ((n + 2 * i + 1) * (n + 2 * i + 2)) as f64;
        coeff_i = -coeff_i; // flip sign
    }

    total
}

/// Get the kth Chebyshev coefficient of x^n. This is just combinatorics.
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
