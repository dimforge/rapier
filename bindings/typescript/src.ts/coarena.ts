export class Coarena<T> {
    fconv: Float64Array;
    uconv: Uint32Array;
    data: Array<T>;
    size: number;

    public constructor() {
        this.fconv = new Float64Array(1);
        this.uconv = new Uint32Array(this.fconv.buffer);
        this.data = new Array<T>();
        this.size = 0;
    }

    public set(handle: number, data: T) {
        let i = this.index(handle);
        while (this.data.length <= i) {
            this.data.push(null);
        }

        if (this.data[i] == null) this.size += 1;
        this.data[i] = data;
    }

    public len(): number {
        return this.size;
    }

    public delete(handle: number) {
        let i = this.index(handle);
        if (i < this.data.length) {
            if (this.data[i] != null) this.size -= 1;
            this.data[i] = null;
        }
    }

    public clear() {
        this.data = new Array<T>();
    }

    public get(handle: number): T | null {
        let i = this.index(handle);
        if (i < this.data.length) {
            return this.data[i];
        } else {
            return null;
        }
    }

    public forEach(f: (elt: T) => void) {
        for (const elt of this.data) {
            if (elt != null) f(elt);
        }
    }

    public getAll(): Array<T> {
        return this.data.filter((elt) => elt != null);
    }

    private index(handle: number): number {
        /// Extracts the index part of a handle (the lower 32 bits).
        /// This is done by first injecting the handle into an Float64Array
        /// which is itself injected into an Uint32Array (at construction time).
        /// The 0-th value of the Uint32Array will become the `number` integer
        /// representation of the lower 32 bits.
        /// Also `this.uconv[1]` then contains the generation number as a `number`,
        /// which we don’t really need.
        this.fconv[0] = handle;
        return this.uconv[0];
    }
}
