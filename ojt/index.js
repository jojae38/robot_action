const { app, BrowserWindow } = require('electron')
// include the Node.js 'path' module at the top of your file
const path = require('path')


// modify your existing createWindow() function
function createWindow() {
    const win = new BrowserWindow({
        width: 400,
        height: 400,
        webPreferences: {

        }
    })
    win.loadFile('index.html')
}
app.whenReady().then(() => {
    createWindow()
})
