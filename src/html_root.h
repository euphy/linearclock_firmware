R"rawText(
<!DOCTYPE html>
<html>
  <head>
    <meta http-equiv='refresh' content='5'/>
    <title>Realtime clock output</title>
    <style>
      body { font-size: 4em;}
    </style>
  </head>
  <body>
    <h1>Real time clock says:</h1>
    <p>%02d:%02d:%02d</p>
    <p>%02d/%02d/%04d</p>
    <img src="/test.svg" />
    <form method="post">
      <input type="datetime-local" name="dateTimeInput" />
      <input type="submit" value="submit" />
    </form>
  </body>
</html>
)rawText
