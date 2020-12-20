# Projeto realizado na seletiva do RobôCin no ano de 2020
### O intuito do projeto é montar um filtro de ruído para tratar as perdas de pacote do ssl-client.
### A metodologia utilizada para reduzir os ruídos foi o filtro de Kalman, que consiste em minimizar a margem de erro de uma medida baseando-se em mesclar um valor predito pelo sistema com o valor recebido da medição.
### O código modificado está na pasta ssl-client e na pasta ssl-viewer está um visualizador de logs gerados no código modificado do ssl-client para facilitar a visualização da eficácia do código alterado.