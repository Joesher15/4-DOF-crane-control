const express = require('express');
const { createProxyMiddleware } = require('http-proxy-middleware');

const app = express();

app.use((req, res, next) => {
    res.header('Access-Control-Allow-Origin', '*');
    res.header('Access-Control-Allow-Methods', 'GET,HEAD,OPTIONS,POST,PUT');
    res.header('Access-Control-Allow-Headers', '*');
    res.header('Cache-Control', 'no-store, no-cache, must-revalidate, proxy-revalidate'),
    res.setHeader('Pragma', 'no-cache');
    res.setHeader('Expires', '0');
    
    next();
});

app.use('/resources', createProxyMiddleware({
    target: 'http://localhost:8080', // Target server URL
    changeOrigin: true,
    pathRewrite: {
        '^/resources': '', // Remove '/urdf' from the request path
    },
}));

app.listen(3000, () => {
    console.log('Proxy server running on http://localhost:3000');
});