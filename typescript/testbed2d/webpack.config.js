const webpack = require("webpack");
const path = require("path");
const CopyPlugin = require("copy-webpack-plugin");

const isDev = process.env.NODE_ENV === "development";
const dist = path.resolve(__dirname, "dist");

/**
 * @type {import('webpack-dev-server').Configuration}
 */
const devServer = {
    static: {
        directory: dist,
    },
    allowedHosts: "all",
    compress: true,
};

/**
 * @type {import('webpack').Configuration}
 */
const webpackConfig = {
    mode: isDev ? "development" : "production",
    entry: "./src/index.ts",
    devtool: "inline-source-map",
    output: {
        path: dist,
        filename: "index.js",
    },
    resolve: {
        extensions: [".ts", ".js"],
    },
    devServer,
    performance: false,
    experiments: {
        asyncWebAssembly: true,
        syncWebAssembly: true,
    },
    module: {
        rules: [
            {
                test: /\.tsx?$/,
                use: "ts-loader",
                exclude: /node_modules/,
            },
        ],
    },
    plugins: [
        new CopyPlugin({
            patterns: [{from: "static", to: dist}],
        }),
    ],
};

module.exports = webpackConfig;
