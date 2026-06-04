// Adapted from https://github.com/dimforge/rapier.js/blob/a86610ffc744efb1541f896cd0f421993e6ddb54/testbed2d/src/Graphics.ts

// DOCUSAURUS: Debug1 start
import * as PIXI from "pixi.js";
import { Viewport } from "pixi-viewport";
// DOCUSAURUS: Debug1 stop

import type * as RAPIER from "@dimforge/rapier2d";

export class Graphics {
    colorIndex: number;
    renderer: PIXI.Renderer;
    scene: PIXI.Container;
    viewport: Viewport;

    constructor() {
        this.renderer = new PIXI.Renderer({
            backgroundColor: 0x292929,
            antialias: false,
            width: window.innerWidth,
            height: window.innerHeight,
        });

        this.scene = new PIXI.Container();
        document.body.appendChild(this.renderer.view);

        this.viewport = new Viewport({
            screenWidth: window.innerWidth,
            screenHeight: window.innerHeight,
            worldWidth: 200,
            worldHeight: 200,
            interaction: this.renderer.plugins.interaction,
        });

        this.scene.addChild(this.viewport);
        this.viewport.drag().pinch().wheel().decelerate();

        this.lines = new PIXI.Graphics();
        this.viewport.addChild(this.lines);

        let me = this;

        function onWindowResize() {
            me.renderer.resize(window.innerWidth, window.innerHeight);
        }

        function onContextMenu(event: UIEvent) {
            event.preventDefault();
        }

        document.oncontextmenu = onContextMenu;
        document.body.oncontextmenu = onContextMenu;

        window.addEventListener("resize", onWindowResize, false);
    }

    lines: PIXI.Graphics;

    // DOCUSAURUS: Debug2 start
    render(world: RAPIER.World) {
        // DOCUSAURUS: Debug0 start
        const { vertices, colors } = world.debugRender();
        // DOCUSAURUS: Debug0 stop

        this.lines.clear();

        for (let i = 0; i < vertices.length / 4; i += 1) {
            let color = PIXI.utils.rgb2hex([
                colors[i * 8],
                colors[i * 8 + 1],
                colors[i * 8 + 2],
            ]);
            this.lines.lineStyle(1.0, color, colors[i * 8 + 3], 0.5, true);
            this.lines.moveTo(vertices[i * 4], -vertices[i * 4 + 1]);
            this.lines.lineTo(vertices[i * 4 + 2], -vertices[i * 4 + 3]);
        }

        this.renderer.render(this.scene);
    }
    // DOCUSAURUS: Debug2 stop

    lookAt(pos: { zoom: number; target: { x: number; y: number } }) {
        this.viewport.setZoom(pos.zoom);
        this.viewport.moveCenter(pos.target.x, pos.target.y);
    }
}