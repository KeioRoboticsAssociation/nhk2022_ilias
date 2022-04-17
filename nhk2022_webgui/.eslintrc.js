/* eslint-disable @typescript-eslint/naming-convention */

module.exports = {
    "env": {
        "browser": true,
        "es2021": true
    },
    "extends": [
        "airbnb-base",
        // "plugin:vue/essential",
        "plugin:vue/vue3-recommended",
        "@vue/typescript/recommended",
        "prettier",
    ],
    "parserOptions": {
        "ecmaVersion": "latest",
        "parser": "@typescript-eslint/parser",
        "sourceType": "module"
    },
    "ignorePatterns": ["*.config.ts"],
    "rules": {
        "import/no-unresolved": "off",
        "import/extensions": "off",
        // console.logを許可
        "no-console": "off",
        // ネーミングルールを追加
        "@typescript-eslint/naming-convention": [
            "error",
            {
                "selector": "default",
                "format": ["camelCase"]
            },
            {
                "selector": [
                "property"
                ],
                "format": [
                "camelCase",
                "PascalCase"
                ]
            },
            {
                "selector": [
                "class",
                "enum",
                "interface",
                "typeAlias",
                "typeParameter"
                ],
                "format": [
                "PascalCase"
                ]
            },
            {
                "selector": "variable",
                "modifiers": [
                "const"
                ],
                "format": [
                "camelCase",
                "UPPER_CASE"
                ]
            }
        ],
    },
}
