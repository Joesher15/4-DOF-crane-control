const ws = new WebSocket("ws://192.168.66.128:8765");

ws.onmessage = (event) => {
    const positions: number[] = JSON.parse(event.data);
    document.getElementById("crane-state")!.innerText = positions.join(", ");
};