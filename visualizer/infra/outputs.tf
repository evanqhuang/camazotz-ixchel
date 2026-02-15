output "pages_url" {
  description = "The default Pages deployment URL"
  value       = "https://${var.project_name}.pages.dev"
}

output "custom_domain" {
  description = "The custom domain URL"
  value       = "https://${var.subdomain}.${var.domain}"
}

output "zone_id" {
  description = "The Cloudflare zone ID"
  value       = local.zone_id
}

output "project_id" {
  description = "The Cloudflare Pages project ID"
  value       = cloudflare_pages_project.visualizer.id
}

output "pages_subdomain" {
  description = "The Cloudflare Pages subdomain"
  value       = cloudflare_pages_project.visualizer.subdomain
}
