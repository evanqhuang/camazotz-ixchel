# Auto-discover the Cloudflare account ID from the API token
data "cloudflare_accounts" "main" {}

locals {
  account_id = data.cloudflare_accounts.main.result[0].id
  zone_id    = data.cloudflare_zones.site.result[0].id
}

# Look up the existing Cloudflare zone for the domain
data "cloudflare_zones" "site" {
  account = {
    id = local.account_id
  }
  name = var.domain
}

# Create the Cloudflare Pages project (direct upload, no git integration)
resource "cloudflare_pages_project" "visualizer" {
  account_id        = local.account_id
  name              = var.project_name
  production_branch = "main"
}

# Attach the map subdomain to the Pages project
resource "cloudflare_pages_domain" "map" {
  account_id   = local.account_id
  project_name = cloudflare_pages_project.visualizer.name
  name         = "${var.subdomain}.${var.domain}"
}

# Create CNAME record for map subdomain pointing to Pages
resource "cloudflare_dns_record" "map" {
  zone_id = local.zone_id
  name    = var.subdomain
  type    = "CNAME"
  content = "${var.project_name}.pages.dev"
  ttl     = 1 # Auto TTL
  proxied = true
  comment = "Dive visualizer on Cloudflare Pages"
}
