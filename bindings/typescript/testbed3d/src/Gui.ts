import GUI from "lil-gui";
import * as Stats from "stats.js";
import type {Testbed} from "./Testbed";

export interface DebugInfos {
    token: number;
    stepId: number;
    worldHash: string;
    worldHashTime: number;
    snapshotTime: number;

    timingStep: number;
    timingCollisionDetection: number;
    timingBroadPhase: number;
    timingNarrowPhase: number;
    timingSolver: number;
    timingVelocityAssembly: number;
    timingVelocityResolution: number;
    timingVelocityUpdate: number;
    timingVelocityWriteback: number;
    timingCcd: number;
    timingCcdToiComputation: number;
    timingCcdBroadPhase: number;
    timingCcdNarrowPhase: number;
    timingCcdSolver: number;
    timingIslandConstruction: number;
    timingUserChanges: number;
}

export class Gui {
    stats: Stats;
    rapierVersion: string;
    maxTimePanelValue: number;
    stepTimePanel: Stats.Panel;
    gui: GUI;
    debugText: HTMLDivElement;

    constructor(testbed: Testbed, simulationParameters: Testbed["parameters"]) {
        // Timings
        this.stats = new Stats();
        this.rapierVersion = testbed.RAPIER.version();
        this.maxTimePanelValue = 16.0;
        this.stepTimePanel = this.stats.addPanel(
            new Stats.Panel("ms (step)", "#ff8", "#221"),
        );
        this.stats.showPanel(this.stats.dom.children.length - 1);
        document.body.appendChild(this.stats.dom);

        var backends = simulationParameters.backends;
        var demos = Array.from(simulationParameters.builders.keys());
        var me = this;

        // For configuring simulation parameters.
        this.gui = new GUI({
            title: "Rapier JS demos",
        });
        var currDemo = this.gui
            .add(simulationParameters, "demo", demos)
            .onChange((demo: string) => {
                testbed.switchToDemo(demo);
            });
        this.gui
            .add(simulationParameters, "numSolverIters", 0, 20)
            .step(1)
            .listen();
        this.gui
            .add(simulationParameters, "debugInfos")
            .listen()
            .onChange((value: boolean) => {
                me.debugText.style.visibility = value ? "visible" : "hidden";
            });
        this.gui.add(simulationParameters, "debugRender").listen();
        this.gui.add(simulationParameters, "running").listen();
        this.gui.add(simulationParameters, "step");
        simulationParameters.step = () => {
            simulationParameters.stepping = true;
        };
        this.gui.add(simulationParameters, "takeSnapshot");
        simulationParameters.takeSnapshot = () => {
            testbed.takeSnapshot();
        };
        this.gui.add(simulationParameters, "restoreSnapshot");
        simulationParameters.restoreSnapshot = () => {
            testbed.restoreSnapshot();
        };
        this.gui.add(simulationParameters, "restart");
        simulationParameters.restart = () => {
            testbed.switchToDemo(currDemo.getValue());
        };

        /*
         * Block of text for debug infos.
         */
        this.debugText = document.createElement("div");
        this.debugText.style.position = "absolute";
        this.debugText.innerHTML = "";
        this.debugText.style.top = 50 + "px";
        this.debugText.style.visibility = "visible";
        this.debugText.style.color = "#fff";
        document.body.appendChild(this.debugText);
    }

    setDebugInfos(infos: DebugInfos) {
        let text = "Version " + this.rapierVersion;
        text += "<br/>[Step " + infos.stepId + "]";

        if (infos.worldHash) {
            text +=
                "<br/>World hash (xxHash128): " + infos.worldHash.toString();
            text +=
                "<br/>World hash time (xxHash128): " +
                infos.worldHashTime +
                "ms";
            text += "<br/>Snapshot time: " + infos.snapshotTime + "ms";
        }

        text += "<br/>timingStep: " + infos.timingStep + "ms";
        text +=
            "<br/>timingCollisionDetection: " +
            infos.timingCollisionDetection +
            "ms";
        text += "<br/>timingBroadPhase: " + infos.timingBroadPhase + "ms";
        text += "<br/>timingNarrowPhase: " + infos.timingNarrowPhase + "ms";
        text += "<br/>timingSolver: " + infos.timingSolver + "ms";
        text +=
            "<br/>timingVelocityAssembly: " +
            infos.timingVelocityAssembly +
            "ms";
        text +=
            "<br/>timingVelocityResolution: " +
            infos.timingVelocityResolution +
            "ms";
        text +=
            "<br/>timingVelocityUpdate: " + infos.timingVelocityUpdate + "ms";
        text +=
            "<br/>timingVelocityWriteback: " +
            infos.timingVelocityWriteback +
            "ms";
        text += "<br/>timingCcd: " + infos.timingCcd + "ms";
        text +=
            "<br/>timingCcdToiComputation: " +
            infos.timingCcdToiComputation +
            "ms";
        text += "<br/>timingCcdBroadPhase: " + infos.timingCcdBroadPhase + "ms";
        text +=
            "<br/>timingCcdNarrowPhase: " + infos.timingCcdNarrowPhase + "ms";
        text += "<br/>timingCcdSolver: " + infos.timingCcdSolver + "ms";
        text +=
            "<br/>timingIslandConstruction: " +
            infos.timingIslandConstruction +
            "ms";
        text += "<br/>timingUserChanges: " + infos.timingUserChanges + "ms";
        this.debugText.innerHTML = text;
    }

    setTiming(timing: number) {
        if (!!timing) {
            this.maxTimePanelValue = Math.max(this.maxTimePanelValue, timing);
            this.stepTimePanel.update(timing, this.maxTimePanelValue);
        }
    }

    resetTiming() {
        this.maxTimePanelValue = 1.0;
        this.stepTimePanel.update(0.0, 16.0);
    }
}
