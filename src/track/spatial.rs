use crate::math::Vec2;
#[derive(Debug, Clone, Copy)]
pub(crate) struct Bounds {
    min: Vec2,
    max: Vec2,
}
impl Bounds {
    pub fn points(points: &[Vec2]) -> Self {
        Self {
            min: Vec2::new(
                points.iter().map(|p| p.x).fold(f64::INFINITY, f64::min),
                points.iter().map(|p| p.y).fold(f64::INFINITY, f64::min),
            ),
            max: Vec2::new(
                points.iter().map(|p| p.x).fold(f64::NEG_INFINITY, f64::max),
                points.iter().map(|p| p.y).fold(f64::NEG_INFINITY, f64::max),
            ),
        }
    }
    fn union(self, b: Self) -> Self {
        Self::points(&[self.min, self.max, b.min, b.max])
    }
    fn distance(self, p: Vec2) -> f64 {
        let dx = (self.min.x - p.x).max(0.).max(p.x - self.max.x);
        let dy = (self.min.y - p.y).max(0.).max(p.y - self.max.y);
        dx.hypot(dy)
    }
}
#[derive(Debug)]
enum Node {
    Leaf {
        bounds: Bounds,
        ids: Vec<usize>,
    },
    Branch {
        bounds: Bounds,
        left: Box<Node>,
        right: Box<Node>,
    },
}
impl Node {
    fn bounds(&self) -> Bounds {
        match self {
            Self::Leaf { bounds, .. } | Self::Branch { bounds, .. } => *bounds,
        }
    }
    fn build(mut ids: Vec<usize>, boxes: &[Bounds]) -> Self {
        let bounds = ids.iter().map(|i| boxes[*i]).reduce(Bounds::union).unwrap();
        if ids.len() <= 8 {
            return Self::Leaf { bounds, ids };
        }
        let x = bounds.max.x - bounds.min.x >= bounds.max.y - bounds.min.y;
        ids.sort_by(|a, b| {
            let center = |i: usize| {
                if x {
                    boxes[i].min.x + boxes[i].max.x
                } else {
                    boxes[i].min.y + boxes[i].max.y
                }
            };
            center(*a).total_cmp(&center(*b))
        });
        let right = ids.split_off(ids.len() / 2);
        Self::Branch {
            bounds,
            left: Box::new(Self::build(ids, boxes)),
            right: Box::new(Self::build(right, boxes)),
        }
    }
    fn nearest(&self, p: Vec2, best: &mut f64, distance: &impl Fn(usize) -> f64) {
        if self.bounds().distance(p) > *best {
            return;
        }
        match self {
            Self::Leaf { ids, .. } => {
                for i in ids {
                    *best = best.min(distance(*i));
                }
            }
            Self::Branch { left, right, .. } => {
                if left.bounds().distance(p) <= right.bounds().distance(p) {
                    left.nearest(p, best, distance);
                    right.nearest(p, best, distance);
                } else {
                    right.nearest(p, best, distance);
                    left.nearest(p, best, distance);
                }
            }
        }
    }
    fn last(&self, p: Vec2, best: &mut Option<usize>, contains: &impl Fn(usize) -> bool) {
        if self.bounds().distance(p) > 1e-12 {
            return;
        }
        match self {
            Self::Leaf { ids, .. } => {
                for i in ids {
                    if best.is_none_or(|b| *i > b) && contains(*i) {
                        *best = Some(*i);
                    }
                }
            }
            Self::Branch { left, right, .. } => {
                left.last(p, best, contains);
                right.last(p, best, contains);
            }
        }
    }
}
#[derive(Debug)]
pub(crate) struct SpatialIndex {
    root: Option<Node>,
}
impl SpatialIndex {
    pub fn new(boxes: Vec<Bounds>) -> Self {
        Self {
            root: if boxes.is_empty() {
                None
            } else {
                Some(Node::build((0..boxes.len()).collect(), &boxes))
            },
        }
    }
    pub fn nearest(&self, p: Vec2, distance: impl Fn(usize) -> f64) -> f64 {
        let mut best = f64::INFINITY;
        if let Some(root) = &self.root {
            root.nearest(p, &mut best, &distance);
        }
        best
    }
    pub fn last(&self, p: Vec2, contains: impl Fn(usize) -> bool) -> Option<usize> {
        let mut best = None;
        if let Some(root) = &self.root {
            root.last(p, &mut best, &contains);
        }
        best
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn indexed_queries_equal_brute_force() {
        let boxes: Vec<_> = (0..1000)
            .map(|i| {
                let p = Vec2::new((i % 31) as f64, (i / 31) as f64);
                Bounds::points(&[p, p + Vec2::new(2., 2.)])
            })
            .collect();
        let tree = SpatialIndex::new(boxes.clone());
        for i in 0..400 {
            let p = Vec2::new(i as f64 * 0.1 - 4., (i % 37) as f64 * 0.9);
            let expected = boxes
                .iter()
                .map(|b| b.distance(p))
                .fold(f64::INFINITY, f64::min);
            assert_eq!(tree.nearest(p, |i| boxes[i].distance(p)), expected);
            assert_eq!(
                tree.last(p, |i| boxes[i].distance(p) == 0.),
                boxes.iter().rposition(|b| b.distance(p) == 0.)
            );
        }
    }
}
