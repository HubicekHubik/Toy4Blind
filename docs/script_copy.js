function copyCode(button) {
    const code = button.parentElement.querySelector('code').innerText;
    navigator.clipboard.writeText(code).then(() => {
        button.innerText = "Zkopírováno!";
        setTimeout(() => button.innerText = "Kopírovat", 2000);
    });
}