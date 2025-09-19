def run_bench(args):
    print(f"[rbe550] Running steps={args.steps}, render_every={args.render_every}, "
          f"grid={args.grid}, fill={args.fill}, enemies={args.enemies}, "
          f"algo={args.algo}, seed={args.seed}, gif={args.gif}, show={args.show}")

def main():
    # Allow `ros2 run rbe550_grid_bench bench_runner`
    class A: pass
    a = A()
    a.steps=200; a.render_every=4; a.grid=64; a.fill=0.20; a.enemies=10; a.algo='bfs'; a.seed=None; a.gif=None; a.show=False
    run_bench(a)
