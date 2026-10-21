---

# AeroDelivery: Tapete de Entrega Aérea Discreta (`aerodelivery-mat-discreta`)

Repositório oficial do projeto **AeroDelivery**, desenvolvido como parte das atividades avaliativas da disciplina de **Matemática Discreta** do curso de Engenharia de Software com Ênfase em Design e Arquitetura da [Jala University].

---

## 👥 Autoria

* **Jenifer Gonçalves da Silva** (STU-1489.BRA.C8)


* **Mateus da Silva Oliveira** (STU-1397.BRA.C7)


* **Roberta Flaviane Soares Paixão** (STU-1497.BRA.C8)


* **Professor Responsável:** Marcos Vinicios Bueno de Morais



---

## 📋 Sumário

* Sobre o Projeto
* Conceitos de Matemática Discreta Aplicados
* Estrutura do Repositório
* Arquitetura e Funcionamento do Sistema
* Tecnologias Utilizadas
* Como Executar o Projeto
* Referências

---

## 💡 Sobre o Projeto

O **AeroDelivery** é um jogo de estratégia e simulação focado na otimização de rotas para entregas de comida utilizando drones urbanos[cite: 3]. O objetivo central do jogador é guiar o drone desde o restaurante de origem até o endereço do cliente selecionado, calculando rotas eficientes que equilibrem distância, tempo e os riscos impostos por um ambiente dinâmico (como clima adverso e obstáculos)[cite: 3, 6].

---

## 🧮 Conceitos de Matemática Discreta Aplicados

O projeto traduz problemas práticos de navegação em modelos matemáticos formais:

1. **Teoria dos Grafos:** O mapa da cidade é modelado como um grafo não direcionado $G = (V, E)$, onde os vértices ($V$) representam as células livres da grade (espaço navegável) e as arestas ($E$) representam as rotas ortogonais possíveis[cite: 9].
2. **Caminhos Mínimos (Dijkstra e $A^*$):** Utilizados para encontrar a rota de custo mínimo entre a posição atual do drone e o destino. O algoritmo $A^*$ emprega a **distância de Manhattan** como heurística admissível e consistente para guiar a busca de forma eficiente[cite: 10, 11].
3. **Cadeias de Markov:** Utilizadas para modelar o comportamento estocástico e dinâmico das condições climáticas (vento, chuva e neblina) ao longo do tempo, permitindo projetar estados futuros através de matrizes de transição probabilísticas[cite: 16].
4. **Filtro de Kalman:** Empregado no tratamento de ruídos nas leituras dos sensores ambientais antes de alimentar a tomada de decisão do agente[cite: 14].

---

## 🗂️ Estrutura do Repositório

| Nome / Arquivo | Descrição |
| --- | --- |
| [`aerodelivery.py`](https://github.com/jeniferGoncalvesDaSilvaDev/aerodelivery-mat-discreta/blob/master/aerodelivery.py) | Script principal contendo a implementação da lógica da simulação e dos algoritmos. |
| `AERODELIVERY_MatemáticaDiscreta.pdf` | Documentação oficial e relatório detalhado do Projeto Capstone. |
| `.vscode/` | Configurações do ambiente de desenvolvimento no Visual Studio Code. |
| `__pycache__/` | Arquivos de cache gerados pelo interpretador Python. |

---

## ⚙️ Arquitetura e Funcionamento do Sistema

### O Pipeline de Incertezas e Navegação

O sistema processa o ambiente em ciclos contínuos de atualização:


$$\text{Sensores} \rightarrow \text{Filtro de Kalman} \rightarrow \text{Cadeia de Markov} \rightarrow \text{Pesos Adaptativos} \rightarrow A^* \rightarrow \text{Melhor Rota}$$


[cite: 14]

* **Pesos das Arestas:** O custo de transição $w(u, v)$ combina o custo estático da célula (célula comum ou zona de risco) com a penalidade climática esperada[cite: 9, 10].
* **Função de Avaliação do $A^*$:** $f(n) = C_{pres}(t) \cdot g(n) + C_{fut}(t) \cdot h(n)$, onde os coeficientes ponderam o custo presente ($C_{pres}$) e a projeção futura ($C_{fut}$) estimada pelas Cadeias de Markov[cite: 11, 17].
* **Política de Replanejamento:** Se ocorrer bloqueio no mapa ou se a variação do custo presente ultrapassar o limiar estipulado ($\tau = 0.12$), a rota é recalculada dinamicamente a partir da posição atual do drone[cite: 7, 8].

---

## 🛠️ Tecnologias Utilizadas

* **[Python (100%)](https://github.com/jeniferGoncalvesDaSilvaDev/aerodelivery-mat-discreta/search?l=python)**: Linguagem de programação principal utilizada para a implementação do motor lógico, estruturas de grafos e simulação.

---

## 🚀 Como Executar

1. **Clone este repositório em sua máquina local:**
```bash
git clone url?id=2.git

```


2. **Acesse o diretório do projeto:**
```bash
cd aerodelivery-mat-discreta

```


3. **Execute o script principal utilizando o Python:**
```bash
python aerodelivery.py

```



---

## 📚 Referências

* CORMEN, Thomas H. et al. **Algoritmos: teoria e prática**. 3. ed. Rio de Janeiro: Elsevier, 2012[cite: 23].
* DIJKSTRA, Edsger W. A note on two problems in connexion with graphs. **Numerische Mathematik**, v. 1, p. 269-271, 1959[cite: 23].
* HART, Peter E.; NILSSON, Nils J.; RAPHAEL, Bertram. A formal basis for the heuristic determination of minimum cost paths. **IEEE Transactions on Systems Science and Cybernetics**, v. 4, n. 2, p. 100-107, 1968[cite: 23].
* ROSEN, Kenneth H. **Matemática discreta e suas aplicações**. 6. ed. São Paulo: McGraw-Hill, 2009[cite: 23].
* RUSSELL, Stuart; NORVIG, Peter. **Inteligência artificial**. 3. ed. Rio de Janeiro: Elsevier, 2013[cite: 23].
