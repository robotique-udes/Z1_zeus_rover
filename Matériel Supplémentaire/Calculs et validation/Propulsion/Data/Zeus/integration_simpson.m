function int_f = integration_simpson(X, F)
%INTÉGRATION NUMÉRIQUE PAR LA MÉTHODE DE SIMPSON
%   QUOI?
%       Sommation des valeurs du sur toute la plage de données.
%      
%   COMMENT?
%       1: Calcul de la longueur
%       2: Calcul des pas d'intégration (H) en fonction des données en X
%       3: Calculs différents pour les premiers et derniers termes
%
%   POURQUOI?
%       Cette méthode est la plus précise et efficace car elle utilise la
%       méthode des courbes polynomiales.
%   
%   PARAMÈTRES
%       Input:  [X]:        La matrice du temps
%               [F]:        La matrice des données à intégrer
%       Output: [int_f]:    La matrice des données intégrées

%% Calcul du pas d'intégration (H)
H = X(2) - X(1);
H = 0.0500;

%% Rénitialisation du régistre ''SOMME''
somme = 0;

%% Boucle for pour traiter chaque donnée unes par unes
    for i=1:1:(length(X))
        
        %Modulo de l'index actuel pour déterminer si il est pair ou non
        modulo = mod(i, 2);
        
        %Si l'index est le premier ou dernier caractère, effectuer le calcul suivant
        if i == 1 || i == length(X)
            somme = somme + abs(F(i));
        else
            
            %Différents calculs si le chiffre est pair (modulo = 0)
            if modulo == 0
                somme = somme + abs(4*F(i)); 
            else
                somme = somme + abs(2*F(i));
            end
        end
    end
    
    %% Retour de la valeur finale calculée
    int_f = (H/3) * somme;
end