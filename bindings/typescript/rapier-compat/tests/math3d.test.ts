import {Vector3, VectorOps} from "../builds/3d-deterministic/pkg";

describe("3d/math", () => {
    test("Vector3", () => {
        const v = new Vector3(0, 1, 2);
        expect(v.x).toBe(0);
        expect(v.y).toBe(1);
        expect(v.z).toBe(2);
    });

    test("VectorOps", () => {
        const v = VectorOps.new(0, 1, 2);
        expect(v.x).toBe(0);
        expect(v.y).toBe(1);
        expect(v.z).toBe(2);
    });
});
