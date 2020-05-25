use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};
use rayon::prelude::*;
use rboids::boid;
use rboids::boid::Boid;

pub fn criterion_benchmark(c: &mut Criterion) {
    let mut group = c.benchmark_group("frame_update");

    for num in [10, 50, 100, 200, 500, 1000].iter() {
        group.throughput(Throughput::Elements(*num as u64));
        group
            .bench_with_input(BenchmarkId::new("sequential", num), num, |b, &num| {
                let mut boids: Vec<Boid> = boid::spawn_boids(&[0.0f32, 0.0, 0.0], 4.0, num);
                let obstacles = boid::create_obstacles();
                b.iter(|| {
                    // test each sample for 100 frames
                    for _ in 0..100 {
                        let copy = boids.clone();
                        boids
                            .iter_mut()
                            .enumerate()
                            .for_each(|(i, bs)| bs.frame_update(i, &copy, &obstacles, 0.02));
                    }
                });
            })
            .sample_size(10); // take 10 samples for each input
        group
            .bench_with_input(BenchmarkId::new("parallel", num), num, |b, &num| {
                let mut boids: Vec<Boid> = boid::spawn_boids(&[0.0f32, 0.0, 0.0], 4.0, num);
                let obstacles = boid::create_obstacles();
                b.iter(|| {
                    // test each sample for 100 frames
                    for _ in 0..100 {
                        let copy = boids.clone();
                        boids
                            .par_iter_mut()
                            .enumerate()
                            .for_each(|(i, bs)| bs.frame_update(i, &copy, &obstacles, 0.02));
                    }
                });
            })
            .sample_size(10); // take 10 samples for each input
    }

    group.finish();
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
