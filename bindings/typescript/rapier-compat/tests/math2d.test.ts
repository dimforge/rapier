import {Vector2, VectorOps} from "../builds/2d-deterministic/pkg";

describe("2d/math", () => {
    test("Vector2", () => {
        const v = new Vector2(0, 1);
        expect(v.x).toBe(0);
        expect(v.y).toBe(1);
    });

    test("VectorOps", () => {
        const v = VectorOps.new(0, 1);
        expect(v.x).toBe(0);
        expect(v.y).toBe(1);
    });
});
