//! Optimal one-to-one matching (Hungarian): tracks to detections (tracker.rs), left boxes to right ones (stereo.rs).

/// Minimum-cost assignment of rows to columns (Hungarian, O(n^2 m)); match[row] = column. Pairs costing more than
/// `gate` are left unmatched.
pub fn assign(cost: &[Vec<f32>], gate: f32) -> Vec<Option<usize>> {
    let n = cost.len();
    let m = cost.first().map_or(0, |r| r.len());
    let mut matched = vec![None; n];
    if n == 0 || m == 0 {
        return matched;
    }
    let flip = n > m; // the algorithm wants rows <= columns
    let (rows, cols) = if flip { (m, n) } else { (n, m) };
    // 1-based; over-gate (and NaN) pairs get a large finite cost so they're picked last; a NaN would never compare
    // below `delta` and the search below would loop forever
    let a = |i: usize, j: usize| {
        let c = if flip { cost[j - 1][i - 1] } else { cost[i - 1][j - 1] };
        if c <= gate { c } else { gate * 100.0 }
    };
    let (mut u, mut v) = (vec![0f32; rows + 1], vec![0f32; cols + 1]);
    let (mut p, mut way) = (vec![0usize; cols + 1], vec![0usize; cols + 1]);
    for i in 1..=rows {
        p[0] = i;
        let mut j0 = 0;
        let mut minv = vec![f32::INFINITY; cols + 1];
        let mut used = vec![false; cols + 1];
        loop {
            used[j0] = true;
            let (i0, mut j1, mut delta) = (p[j0], 0, f32::INFINITY);
            for j in 1..=cols {
                if !used[j] {
                    let cur = a(i0, j) - u[i0] - v[j];
                    if cur < minv[j] {
                        minv[j] = cur;
                        way[j] = j0;
                    }
                    if minv[j] < delta {
                        delta = minv[j];
                        j1 = j;
                    }
                }
            }
            for j in 0..=cols {
                if used[j] {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
            if p[j0] == 0 {
                break;
            }
        }
        loop {
            let j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
            if j0 == 0 {
                break;
            }
        }
    }
    for (j, &pj) in p.iter().enumerate().skip(1) {
        if pj == 0 {
            continue;
        }
        let (row, col) = if flip { (j - 1, pj - 1) } else { (pj - 1, j - 1) };
        if cost[row][col] <= gate {
            matched[row] = Some(col);
        }
    }
    matched
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn assignment_is_optimal_not_greedy() {
        // Greedy takes the cheapest pair (0,0)=1 and is left with (1,1)=10; optimum is 2+2.
        assert_eq!(assign(&[vec![1.0, 2.0], vec![2.0, 10.0]], 100.0), [Some(1), Some(0)]);
    }

    #[test]
    fn assignment_rectangular_gated() {
        let m = assign(&[vec![0.1, 5.0], vec![5.0, 5.0], vec![5.0, 0.2]], 1.0); // more rows than columns
        assert_eq!(m, [Some(0), None, Some(1)]);
    }

    #[test]
    fn assignment_nan_cost_is_gated() {
        assert_eq!(assign(&[vec![f32::NAN]], 1.0), [None]); // used to hang
    }
}
